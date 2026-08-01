"""Shared error types for the physical navigation runtime components."""


class PhysicalNavigationRuntimeError(RuntimeError):
    def __init__(self, code: str, message: str, *, primary_error=None):
        self.code = code
        self.primary_error = primary_error
        super().__init__(message)


class EpisodeCancelled(Exception):
    def __init__(self, stage: str):
        self.stage = stage
        super().__init__(stage)
