"""
Behaviors Package
Exports the factory and base class for easy access.
"""
from .base import BaseBehavior
from .factory import BehaviorFactory

__all__ = [
    'BaseBehavior',
    'BehaviorFactory'
]