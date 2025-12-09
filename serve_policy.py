'''

export JAX_PLATFORM_NAME=cuda
export JAX_PLATFORMS=cuda

python serve_policy.py \
  --port 8000 \
  policy:checkpoint \
  --policy.config=pi0_droid_low_mem_finetune \
  --policy.dir=/home/ilovejafari/openpi/scripts/checkpoints/pi0_droid_low_mem_finetune/mycobot_run/99
  
    --default_prompt="pick up the red block" \
'''

import argparse
import dataclasses
import enum
import itertools
import logging
import socket

try:  # pragma: no cover - tyro is optional at runtime
    import tyro
except ModuleNotFoundError:  # pragma: no cover - fallback when tyro is unavailable
    tyro = None

from openpi.policies import policy as _policy
from openpi.policies import policy_config as _policy_config
from openpi.serving import websocket_policy_server
from openpi.training import config as _config


class EnvMode(enum.Enum):
    """Supported environments."""

    ALOHA = "aloha"
    ALOHA_SIM = "aloha_sim"
    DROID = "droid"
    LIBERO = "libero"


@dataclasses.dataclass
class Checkpoint:
    """Load a policy from a trained checkpoint."""

    # Training config name (e.g., "pi0_aloha_sim").
    config: str
    # Checkpoint directory (e.g., "checkpoints/pi0_aloha_sim/exp/10000").
    dir: str


@dataclasses.dataclass
class Default:
    """Use the default policy for the given environment."""


@dataclasses.dataclass
class Args:
    """Arguments for the serve_policy script."""

    # Environment to serve the policy for. This is only used when serving default policies.
    env: EnvMode = EnvMode.ALOHA_SIM

    # If provided, will be used in case the "prompt" key is not present in the data, or if the model doesn't have a default
    # prompt.
    default_prompt: str | None = None
    default_prompt_2: str | None = None
    default_prompt_3: str | None = None

    # Port to serve the policy on.
    port: int = 8000
    # Record the policy's behavior for debugging.
    record: bool = False

    # Specifies how to load the policy. If not provided, the default policy for the environment will be used.
    policy: Checkpoint | Default = dataclasses.field(default_factory=Default)


# Default checkpoints that should be used for each environment.
DEFAULT_CHECKPOINT: dict[EnvMode, Checkpoint] = {
    EnvMode.ALOHA: Checkpoint(
        config="pi05_aloha",
        dir="gs://openpi-assets/checkpoints/pi05_base",
    ),
    EnvMode.ALOHA_SIM: Checkpoint(
        config="pi0_aloha_sim",
        dir="gs://openpi-assets/checkpoints/pi0_aloha_sim",
    ),
    EnvMode.DROID: Checkpoint(
        config="pi05_droid",
        dir="gs://openpi-assets/checkpoints/pi05_droid",
    ),
    EnvMode.LIBERO: Checkpoint(
        config="pi05_libero",
        dir="gs://openpi-assets/checkpoints/pi05_libero",
    ),
}


def create_default_policy(env: EnvMode, *, default_prompt: str | None = None) -> _policy.Policy:
    """Create a default policy for the given environment."""
    if checkpoint := DEFAULT_CHECKPOINT.get(env):
        return _policy_config.create_trained_policy(
            _config.get_config(checkpoint.config), checkpoint.dir, default_prompt=default_prompt
        )
    raise ValueError(f"Unsupported environment mode: {env}")


def _gather_prompts(args: Args) -> list[str]:
    prompts = []
    for option in (args.default_prompt, args.default_prompt_2, args.default_prompt_3):
        if option is None:
            continue
        stripped = option.strip()
        if stripped:
            prompts.append(stripped)
    return prompts


def create_policy(args: Args, default_prompt: str | None) -> _policy.Policy:
    """Create a policy from the given arguments."""
    match args.policy:
        case Checkpoint():
            return _policy_config.create_trained_policy(
                _config.get_config(args.policy.config), args.policy.dir, default_prompt=default_prompt
            )
        case Default():
            return create_default_policy(args.env, default_prompt=default_prompt)


class PromptFallbackPolicy(_policy.BasePolicy):
    """Injects fallback prompts when the client does not send one."""

    def __init__(self, policy: _policy.BasePolicy, prompts: list[str]):
        self._policy = policy
        self._prompts = prompts
        self._prompt_cycle = itertools.cycle(prompts) if prompts else None

    def infer(self, obs: dict, *, noise=None):
        if self._prompt_cycle is not None and not obs.get("prompt"):
            obs = dict(obs)
            obs["prompt"] = next(self._prompt_cycle)
        if noise is not None:
            return self._policy.infer(obs, noise=noise)
        return self._policy.infer(obs)

    @property
    def metadata(self):
        metadata = getattr(self._policy, "metadata", {}) or {}
        if not self._prompts:
            return metadata
        return {**metadata, "default_prompts": list(self._prompts)}

    def __getattr__(self, name):
        return getattr(self._policy, name)


def main(args: Args) -> None:
    prompt_choices = _gather_prompts(args)
    default_prompt = prompt_choices[0] if prompt_choices else None
    policy = create_policy(args, default_prompt)
    if prompt_choices:
        policy = PromptFallbackPolicy(policy, prompt_choices)
    policy_metadata = policy.metadata

    # Record the policy's behavior.
    if args.record:
        policy = _policy.PolicyRecorder(policy, "policy_records")

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    logging.info("Creating server (host: %s, ip: %s)", hostname, local_ip)

    server = websocket_policy_server.WebsocketPolicyServer(
        policy=policy,
        host="0.0.0.0",
        port=args.port,
        metadata=policy_metadata,
    )
    server.serve_forever()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, force=True)
    if tyro is not None:
        main(tyro.cli(Args))
    else:
        parser = argparse.ArgumentParser(description="Serve an OpenPI policy without tyro installed.")
        parser.add_argument(
            "policy_subcommand",
            nargs="?",
            help="Use 'policy:checkpoint' to load a custom checkpoint (default policies otherwise).",
        )
        env_choices = {mode.name.lower(): mode for mode in EnvMode}
        parser.add_argument("--env", choices=env_choices.keys(), default="aloha_sim")
        parser.add_argument("--default_prompt")
        parser.add_argument("--default_prompt_2")
        parser.add_argument("--default_prompt_3")
        parser.add_argument("--port", type=int, default=8000)
        parser.add_argument("--record", action="store_true")
        parser.add_argument("--policy.config", dest="policy_config")
        parser.add_argument("--policy.dir", dest="policy_dir")
        parsed = parser.parse_args()

        env = env_choices[parsed.env]
        args = Args(
            env=env,
            default_prompt=parsed.default_prompt,
            default_prompt_2=parsed.default_prompt_2,
            default_prompt_3=parsed.default_prompt_3,
            port=parsed.port,
            record=parsed.record,
        )

        use_checkpoint = parsed.policy_subcommand == "policy:checkpoint" or (
            parsed.policy_config is not None or parsed.policy_dir is not None
        )
        if use_checkpoint:
            if parsed.policy_config is None or parsed.policy_dir is None:
                parser.error("--policy.config and --policy.dir are required when using policy:checkpoint")
            args.policy = Checkpoint(config=parsed.policy_config, dir=parsed.policy_dir)

        main(args)
