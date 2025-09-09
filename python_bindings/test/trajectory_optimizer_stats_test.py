from pyidto import TrajectoryOptimizerStats


def test_optimizer_stats():
    """Smoke test for an IDTO optimizer stats."""
    stats = TrajectoryOptimizerStats()
    assert stats.is_empty()
