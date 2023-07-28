import rospy

from analog_gauge_reader.msg import GaugeReading, GaugeReadings
from analog_gauge_reader.srv import (
    GaugeReader as GaugeReaderSrv,
    GaugeReaderRequest,
    GaugeReaderResponse,
)

from moma_mission.core import StateRos


class GaugeReaderState(StateRos):
    """
    Call a detection service to read a gauge
    """

    def __init__(self, ns):
        StateRos.__init__(
            self,
            ns=ns,
            outcomes=["Completed", "NextDetection", "Failure"],
            input_keys=["continue_gauge_reading"],
        )
        self.gauge_reader_srv_client = rospy.ServiceProxy(
            self.get_scoped_param("gauge_reader_topic", "analog_gauge_reader/read"),
            GaugeReaderSrv,
        )

        self.min_successful_readings = self.get_scoped_param("min_successful_readings")

        self.successful_readings = None
        self.readings = None
        self.reading_pub = rospy.Publisher("/gauges_read", GaugeReading, queue_size=1)

    def init(self):
        self.successful_readings = 0
        self.readings = []

    def _request_reading(self):
        try:
            self.perception_srv_client.wait_for_service(timeout=10)
        except rospy.ROSException as exc:
            rospy.logwarn(
                "Service {} not available yet".format(
                    self.perception_srv_client.resolved_name
                )
            )
            return False

        req = GaugeReaderRequest()
        try:
            res = self.gauge_reader_srv_client.call(req)
        except Exception as e:
            rospy.logerr(e)
            return False

        rospy.loginfo(
            f"New gauge reading is {res.result[0].value} {res.result[0].unit}"
        )

        self.readings.append(res.result[0])
        return True

    def run_with_userdata(self, userdata):
        if not userdata.continue_gauge_reading:
            self.init()

        if self._request_reading():
            self.successful_detections += 1
        else:
            rospy.logwarn("Failed to read gauge")

        rospy.loginfo(
            f"Current number of successful gauge readings: {self.successful_detections}"
        )

        if self.successful_detections < self.min_successful_readings:
            return "NextDetection"

        # TODO publish reading
        rospy.loginfo("Gauge reading completed successfully, final reading is ...")
        self.reading_publisher.publish(self.readings[0])
        rospy.sleep(2.0)
        return "Completed"
