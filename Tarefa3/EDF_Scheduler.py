from simso.core.Scheduler import SchedulerInfo
from EDF_mono import EDF_mono
from simso.utils import PartitionedScheduler


class P_EDF(PartitionedScheduler):
    def init(self):
        PartitionedScheduler.init(self, SchedulerInfo("EDF_mono", EDF_mono))

    def packer(self):
        # First Fit
        cpus = [[cpu, 0] for cpu in self.processors]
        for task in self.task_list:
            j = 0
            # Find a processor with free space.
            while cpus[j][1] + float(task.wcet) / task.period > 1.0:
                j += 1
                if j >= len(self.processors):
                    print("oops bin packing failed.")
                    return False

            # Affect it to the task.
            self.affect_task_to_processor(task, cpus[j][0])

            # Update utilization.
            cpus[j][1] += float(task.wcet) / task.period
        return True

