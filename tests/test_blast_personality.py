import unittest

from robot_agent.blast_personality import (
    BLAST_PERSONA_BY_LOCALE,
    EV3_PERSONA_BY_LOCALE,
    MAX_PERSONA_CHARS,
    normalize_persona_by_locale,
)


class BlastPersonalityTests(unittest.TestCase):
    def test_ev3_persona_is_shared_grumpy_and_wording_only(self):
        self.assertEqual(normalize_persona_by_locale(EV3_PERSONA_BY_LOCALE), EV3_PERSONA_BY_LOCALE)
        self.assertIn("grumpy", EV3_PERSONA_BY_LOCALE["en"])
        self.assertIn("swear occasionally", EV3_PERSONA_BY_LOCALE["en"])
        self.assertIn("never change facts, actions", EV3_PERSONA_BY_LOCALE["en"])
        self.assertIn("svär ibland", EV3_PERSONA_BY_LOCALE["sv"])

    def test_persona_is_energetic_theatrical_harmless_and_text_only(self):
        required_phrases = {
            "sv": (
                "överpeppad",
                "charmigt halvgalen",
                "EV3:s buttra pessimism",
                "lätta våldsamhet är enbart teatralisk ordlek",
                "aldrig människor eller djur",
                "endast språkstil",
                (
                    "aldrig påverka handlingar, säkerhet, fakta, "
                    "sensorbedömningar eller beslut"
                ),
            ),
            "en": (
                "overhyped",
                "lovably half-mad",
                "EV3's grumpy pessimism",
                "mild violence is theatrical wordplay",
                "never people or animals",
                "text style only",
                (
                    "never affect actions, safety, facts, sensor assessments, "
                    "or decisions"
                ),
            ),
        }

        self.assertEqual(set(BLAST_PERSONA_BY_LOCALE), set(required_phrases))
        for locale, phrases in required_phrases.items():
            persona = BLAST_PERSONA_BY_LOCALE[locale]
            with self.subTest(locale=locale):
                self.assertLessEqual(len(persona), MAX_PERSONA_CHARS)
                for phrase in phrases:
                    self.assertIn(phrase, persona)


if __name__ == "__main__":
    unittest.main()
