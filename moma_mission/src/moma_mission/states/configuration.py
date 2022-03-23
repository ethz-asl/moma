import dynamic_reconfigure.client

from moma_mission.core import StateRosControl


class DynamicReconfigureState(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)

        self.client = dynamic_reconfigure.client.Client(self.get_scoped_param("node"))


    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        #try:
        self.client.update_configuration(self.get_scoped_param("params"))
        #except:
        #    return 'Failure'

        return 'Completed'
