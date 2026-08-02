"""Shared validation for LM Studio chat-completion endpoints."""

import ipaddress
from urllib.parse import urlsplit


OPENAI_V1_CHAT_COMPLETIONS_PATH = "/v1/chat/completions"
LM_STUDIO_V0_CHAT_COMPLETIONS_PATH = "/api/v0/chat/completions"
CHAT_COMPLETIONS_PATHS = frozenset((
    OPENAI_V1_CHAT_COMPLETIONS_PATH,
    LM_STUDIO_V0_CHAT_COMPLETIONS_PATH,
))

_RFC1918_NETWORKS = tuple(
    ipaddress.ip_network(value)
    for value in (
        "10.0.0.0/8",
        "172.16.0.0/12",
        "192.168.0.0/16",
    )
)
_PRIVATE_IPV6_NETWORK = ipaddress.ip_network("fc00::/7")


class LMStudioEndpointError(ValueError):
    """An LM Studio endpoint or model identifier is outside host policy."""


def _private_lan_address(address) -> bool:
    if address.version == 4:
        return any(address in network for network in _RFC1918_NETWORKS)
    return address in _PRIVATE_IPV6_NETWORK


def validate_lm_studio_base_url(
    value: str,
    *,
    allow_private_lan: bool = False,
) -> str:
    try:
        parsed = urlsplit(value)
        hostname = parsed.hostname
        parsed.port
    except (AttributeError, ValueError):
        raise LMStudioEndpointError("LM Studio base URL is invalid") from None
    if (
        parsed.scheme not in ("http", "https")
        or not parsed.netloc
        or parsed.username is not None
        or parsed.password is not None
        or parsed.path not in ("", "/")
        or parsed.query
        or parsed.fragment
        or hostname is None
    ):
        raise LMStudioEndpointError("LM Studio base URL is invalid")
    if not isinstance(allow_private_lan, bool):
        raise LMStudioEndpointError("LM Studio base URL is invalid")
    if hostname.lower() != "localhost":
        try:
            address = ipaddress.ip_address(hostname)
            if not address.is_loopback and not (
                allow_private_lan and _private_lan_address(address)
            ):
                raise ValueError
        except ValueError:
            raise LMStudioEndpointError(
                "LM Studio must use an allowed numeric address"
            ) from None
    return value.rstrip("/")


def validate_lm_studio_model_id(value: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 200
        or any(ord(character) < 32 for character in value)
    ):
        raise LMStudioEndpointError("LM Studio model id is invalid")
    return value


__all__ = (
    "CHAT_COMPLETIONS_PATHS",
    "LMStudioEndpointError",
    "LM_STUDIO_V0_CHAT_COMPLETIONS_PATH",
    "OPENAI_V1_CHAT_COMPLETIONS_PATH",
    "validate_lm_studio_base_url",
    "validate_lm_studio_model_id",
)
