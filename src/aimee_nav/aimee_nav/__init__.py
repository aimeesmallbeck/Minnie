try:
    from aimee_nav._core import (
        GridMap, ScanMatcher, EKF2D, PoseGraph, Keyframe, Constraint,
        GlobalPlanner, DWALocalPlanner, DWAConfig,
        MCL2D, FrontierDetector, FrontierCluster,
    )
except ImportError as e:
    import warnings
    warnings.warn(f"C++ extension not available: {e}")
    GridMap = None
    ScanMatcher = None
    EKF2D = None
    PoseGraph = None
    Keyframe = None
    Constraint = None
    GlobalPlanner = None
    DWALocalPlanner = None
    DWAConfig = None
    MCL2D = None
    FrontierDetector = None
    FrontierCluster = None
