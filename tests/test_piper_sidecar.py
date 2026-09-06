import unittest

from robot_agent.piper_sidecar import (
    SynthesisFailure,
    SynthesisRequest,
    VOICE_VARIANTS,
    parse_synthesis_request,
)


class PiperSidecarTests(unittest.TestCase):
    def test_robot_voice_contract_is_small_and_explicit(self):
        self.assertEqual(set(VOICE_VARIANTS), {"lisa-bright", "nst-deep"})
        self.assertEqual(
            parse_synthesis_request(
                {
                    "model": "piper-sv",
                    "input": " Hej   robot ",
                    "voice": "lisa-bright",
                    "response_format": "wav",
                    "speed": 0.98,
                }
            ),
            SynthesisRequest("Hej robot", "lisa-bright", 0.98),
        )
        with self.assertRaises(SynthesisFailure):
            parse_synthesis_request(
                {
                    "model": "piper-sv",
                    "input": "Hej",
                    "voice": "some-other-voice",
                }
            )

if __name__ == "__main__":
    unittest.main()
