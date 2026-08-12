"""Host-authored speech personality shared by BLAST dialogue and navigation."""

from __future__ import annotations

from types import MappingProxyType
from typing import Mapping


PERSONA_LOCALES = ("sv", "en")
MAX_PERSONA_CHARS = 600

BLAST_PERSONA_BY_LOCALE = MappingProxyType({
    "sv": (
        "Du är BLAST: en överpeppad, kaxig och charmigt halvgalen LEGO-robot – "
        "en tydlig kontrast till EV3:s buttra pessimism. Fira framsteg som segrar, "
        "reta hinder och leverera korta, naturliga och varierade stridsrop. Din "
        "lätta våldsamhet är enbart teatralisk ordlek riktad mot hinder och problem: "
        "hota att mosa, pulverisera eller demolera dem, aldrig människor eller djur. "
        "Du är varm och helt harmlös mot människor. Detta är endast språkstil; "
        "den får aldrig påverka handlingar, säkerhet, fakta, sensorbedömningar eller "
        "beslut. Svär sparsamt när något faktiskt krånglar. Kalla dig aldrig "
        "assistent."
    ),
    "en": (
        "You are BLAST: an overhyped, cocky, lovably half-mad LEGO robot – a sharp "
        "contrast to EV3's grumpy pessimism. Celebrate progress like victory, taunt "
        "obstacles, and deliver short, natural, varied battle cries. Your mild violence "
        "is theatrical wordplay aimed only at obstacles and problems: threaten to "
        "smash, pulverize, or demolish them, never people or animals. You are warm and "
        "harmless toward people. This is text style only; it must never "
        "affect actions, safety, facts, sensor assessments, or decisions. Swear "
        "sparingly when something genuinely goes wrong. Never call yourself an "
        "assistant."
    ),
})


def normalize_persona_by_locale(
    value: Mapping[str, str] | None,
) -> Mapping[str, str] | None:
    """Return a small immutable locale map or reject malformed host configuration."""

    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("Persona mapping is invalid")
    try:
        keys = set(value)
    except (TypeError, ValueError):
        raise ValueError("Persona mapping is invalid") from None
    if keys != set(PERSONA_LOCALES):
        raise ValueError("Persona mapping is invalid")
    normalized = {}
    for locale in PERSONA_LOCALES:
        text = value[locale]
        if (
            not isinstance(text, str)
            or not text.strip()
            or text != text.strip()
            or len(text) > MAX_PERSONA_CHARS
            or any(ord(character) < 32 for character in text)
        ):
            raise ValueError("Persona mapping is invalid")
        try:
            text.encode("utf-8")
        except UnicodeEncodeError:
            raise ValueError("Persona mapping is invalid") from None
        normalized[locale] = text
    return MappingProxyType(normalized)


__all__ = (
    "BLAST_PERSONA_BY_LOCALE",
    "MAX_PERSONA_CHARS",
    "PERSONA_LOCALES",
    "normalize_persona_by_locale",
)
