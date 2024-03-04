import numpy as np
from stonesoup.reader.base import DetectionReader
from stonesoup.buffered_generator import BufferedGenerator
from stonesoup.types.detection import Detection
from stonesoup.models.transition.linear import (
    CombinedLinearGaussianTransitionModel,
    ConstantVelocity,
    LinearGaussianTimeInvariantTransitionModel,
)
from stonesoup.models.measurement.linear import LinearGaussian
from stonesoup.predictor.kalman import KalmanPredictor
from stonesoup.updater.kalman import KalmanUpdater
from stonesoup.hypothesiser.distance import DistanceHypothesiser
from stonesoup.measures import Mahalanobis
from stonesoup.dataassociator.neighbour import GNNWith2DAssignment
from stonesoup.deleter.error import CovarianceBasedDeleter
from stonesoup.initiator.simple import MultiMeasurementInitiator
from stonesoup.types.state import GaussianState
from stonesoup.tracker.simple import MultiTargetTracker
from stonesoup.base import Property


class BboxDetector(DetectionReader):
    detections = set()
    ts = None

    def add_detection(self, centroid, label, ts):
        self.detections.add(Detection(state_vector=centroid, timestamp=ts, metadata={"label": label}))
        self.ts = ts

    @BufferedGenerator.generator_method
    def detections_gen(self):
        if self.ts is not None:
            yield self.ts, self.detections
            self.detections = set()
            self.ts = None


class FilteredMultiMeasurementInitiator(MultiMeasurementInitiator):
    initiable_labels: list = Property(default=None, doc="Labels which are allowed to initiate a new track")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def initiate(self, detections, timestamp, **kwargs):
        def filter(d, l):
            label = d.metadata.get("label")
            return label is not None and label in l

        detections = set([d for d in detections if filter(d, self.initiable_labels)])
        return super().initiate(detections, timestamp, **kwargs)


class Tracker:
    def __init__(self, initiable_labels=[1]):
        self.transition_model = CombinedLinearGaussianTransitionModel(
            [
                ConstantVelocity(4),
                ConstantVelocity(4),
                ConstantVelocity(2),
                LinearGaussianTimeInvariantTransitionModel(transition_matrix=np.eye(3), covariance_matrix=np.eye(3) * 0.2),
            ]
        )
        self.measurement_model = LinearGaussian(
            ndim_state=9,
            mapping=(0, 2, 4, 6, 7, 8),
            noise_covar=np.diag([1, 1, 1, 1.5, 1.5, 1.5]) * 0.05,
        )
        self.updater = KalmanUpdater(self.measurement_model)
        self.predictor = KalmanPredictor(self.transition_model)
        self.hypothesizer = DistanceHypothesiser(
            self.predictor,
            self.updater,
            measure=Mahalanobis(),
            missed_distance=1.5,
        )
        self.associator = GNNWith2DAssignment(self.hypothesizer)
        self.deleter = CovarianceBasedDeleter(covar_trace_thresh=5)
        self.initiator = FilteredMultiMeasurementInitiator(
            prior_state=GaussianState(
                [[0], [0], [0], [0], [0], [0], [0], [0], [0]],
                np.diag([1, 100, 1, 100, 1, 100, 1, 1, 1]),
            ),
            initiable_labels=initiable_labels,
            measurement_model=self.measurement_model,
            deleter=CovarianceBasedDeleter(covar_trace_thresh=35),  # be more lenient to allow covar to converge
            data_associator=self.associator,
            updater=self.updater,
            min_points=3,
        )
        self.detector = BboxDetector()
        self.tracker = MultiTargetTracker(self.initiator, self.deleter, self.detector, self.associator, self.updater)

    def add_detection(self, centroid, label, ts):
        self.detector.add_detection(centroid, label, ts)

    def get_tracker(self):
        return self.tracker
