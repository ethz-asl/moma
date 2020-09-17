import os
import pickle
from collections import OrderedDict
from datetime import datetime
from copy import deepcopy

from highlevel_planning.knowledge.knowledge_base import KnowledgeBase
from highlevel_planning.tools.config import ConfigYaml


class Reporter:
    def __init__(self, basedir: str, config: ConfigYaml):
        self.basedir = basedir
        self.data = dict()
        self.metrics = OrderedDict()
        self.time_stamp = datetime.now()
        self.metrics["time"] = self.time_stamp.strftime("%Y-%m-%d_%H-%M-%S")
        self.metrics["description"] = input("Experiment description: ")
        self.data["configuration"] = deepcopy(config._cfg)

    def _extract_kb_metrics(self, kb, prefix: str):
        self.metrics[f"{prefix}_actions"] = str(len(kb.actions))
        self.metrics[f"{prefix}_types"] = str(len(kb.types))
        self.metrics[f"{prefix}_objects"] = str(len(kb.objects))
        self.metrics[f"{prefix}_visible_objects"] = str(len(kb.visible_objects))
        self.metrics[f"{prefix}_object_predicates"] = str(len(kb.object_predicates))
        self.metrics[f"{prefix}_initial_state_predicates"] = str(
            len(kb.initial_state_predicates)
        )
        self.metrics[f"{prefix}_lookup_table"] = str(len(kb.lookup_table))
        self.metrics[f"{prefix}_parameterizations"] = str(len(kb.parameterizations))
        self.metrics[f"{prefix}_meta_actions"] = str(len(kb.meta_actions))

    def report_before_exploration(self, knowledge_base: KnowledgeBase, plan):
        kb_clone = KnowledgeBase(self.basedir)
        kb_clone.duplicate(knowledge_base)
        self.data["kb_before"] = kb_clone
        self._extract_kb_metrics(kb_clone, "kb_before")
        self.data["plan_before"] = deepcopy(plan)
        self.metrics["plan_before_success"] = str(True if plan is not False else False)

    def report_after_exploration(
        self, knowledge_base: KnowledgeBase, exploration_metrics: OrderedDict
    ):
        kb_clone = KnowledgeBase(self.basedir)
        kb_clone.duplicate(knowledge_base)
        self.data["kb_after"] = kb_clone
        self._extract_kb_metrics(kb_clone, "kb_after")
        for key, value in exploration_metrics.items():
            self.metrics[f"exp_{key}"] = value

    def report_after_planning(self, plan):
        self.data["plan_after"] = deepcopy(plan)
        self.metrics["plan_after_success"] = True if plan is not False else False

    def write_result_file(self):
        savedir = os.path.join(self.basedir, "data", "reports")
        if not os.path.isdir(savedir):
            os.makedirs(savedir)

        # Write index file
        savefile = os.path.join(savedir, f"{self.time_stamp}_index.txt")
        with open(savefile, "w") as f:
            for key, value in self.metrics.items():
                f.write(f"{key:36}: {value}\n")

        # Write data
        with open(os.path.join(savedir, f"{self.time_stamp}_data.pkl"), "wb") as f:
            pickle.dump(self.data, f)
