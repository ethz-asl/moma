import rospy
import numpy as np

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

        self.readings = None
        self.reading_pub = rospy.Publisher("/gauges_read", GaugeReading, queue_size=1)

    def init(self):
        self.readings = []

    def _request_reading(self):
        try:
            self.gauge_reader_srv_client.wait_for_service(timeout=10)
        except rospy.ROSException as exc:
            rospy.logwarn(
                "Service {} not available yet".format(
                    self.gauge_reader_srv_client.resolved_name
                )
            )
            return False

        req = GaugeReaderRequest()
        try:
            res = self.gauge_reader_srv_client.call(req)
        except Exception as e:
            rospy.logerr(e)
            return False

        if len(res.result.readings) > 1:
            rospy.logwarn(
                "Detected more than one gauge in the image, using the first one that got detected"
            )

        rospy.loginfo(
            f"New gauge reading is {res.result.readings[0].value.data} {res.result.readings[0].unit.data}"
        )

        self.readings.append(res.result.readings[0])
        return True

    def run_with_userdata(self, userdata):
        if not userdata.continue_gauge_reading:
            self.init()

        if not self._request_reading():
            rospy.logwarn("Failed to read gauge")

        rospy.loginfo(f"Current successful gauge readings:\n{self.readings}")
        rospy.loginfo(
            f"Current number of successful gauge readings: {len(self.readings)}"
        )

        if len(self.readings) < self.min_successful_readings:
            return "NextDetection"

        final_reading = GaugeReading()

        values = [reading.value.data for reading in self.readings]
        final_reading.value.data = np.median(values)

        units = set([reading.unit.data for reading in self.readings])
        if "" in units:
            units.remove("")
        final_reading.unit.data = ""
        if len(units) == 1:
            final_reading.unit.data = units.pop()

        rospy.loginfo(
            f"Gauge reading completed successfully, final reading is {final_reading.value.data} {final_reading.unit.data}"
        )
        self.reading_pub.publish(final_reading)
        rospy.sleep(2.0)
        return "Completed"
