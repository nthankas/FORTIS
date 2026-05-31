"""Shared QoS profiles for FORTIS state topics.

Centralizes the "latched" profile used for the mission_state contract and
other state-on-connect topics, so every publisher and subscriber stays
byte-identical. A QoS mismatch between a publisher and a subscriber makes
DDS silently refuse the connection, so this is the single source of truth --
do not hand-roll the profile at call sites.
"""

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def latched_qos_profile(depth: int = 1) -> QoSProfile:
    """Return the FORTIS latched QoS: TRANSIENT_LOCAL + RELIABLE, depth=1.

    "Latched" means a subscriber that connects after the publisher already
    sent its most recent message still receives that value on connect,
    rather than waiting for the next update. Used for slow-changing state
    topics (mission_state, drive armed / drive_healthy, event-console
    context) where a late joiner must immediately learn the current state.
    """
    return QoSProfile(
        depth=depth,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )
