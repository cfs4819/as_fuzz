import carla
import threading
from agents.navigation.behavior_agent import BehaviorAgent

class NpcBase(object):
    def __init__(self,
                 start_loc,
                 end_loc,
                 blueprint: carla.ActorBlueprint,
                 start_time):

        self.start_loc: carla.Transform = start_loc
        self.end_loc: carla.Transform = end_loc
        self.blueprint: carla.ActorBlueprint = blueprint
        self.waypoints = []
        self.start_time = start_time

        self.is_running = False
        self.reached_destination = False
        self.control_thread: threading.Thread = None
        self.close_event: threading.Event = None


class NpcVehicle(NpcBase):
    '''
        choose your blueprints by random.choice(Scenario.vehicle_blueprint)
    '''

    def __init__(self,
                 start_loc: carla.Transform,
                 end_loc: carla.Transform,
                 blueprint: carla.ActorBlueprint,
                 start_time,
                 behavior_type: int,
                 agent_type: str,
                 start_speed: float = 0.0,
                 vehicle_id=None):
        super(NpcVehicle, self).__init__(
            start_loc=start_loc,
            end_loc=end_loc,
            blueprint=blueprint,
            start_time=start_time
        )
        self.behavior_type: int = behavior_type  # 0: driving, 1: starting, 2: parked
        self.agent_type: str = agent_type
        self.agent: BehaviorAgent = None
        self.vehicle = None  # the actor object
        self.vehicle_id: str = vehicle_id
        self.start_speed: float = start_speed

class NpcWalker(NpcBase):
    def __init__(self,
                 start_loc: carla.Transform,
                 end_loc: carla.Transform,
                 blueprint: carla.ActorBlueprint,
                 start_time,
                 max_speed: float,
                 behavior_type: int = 0,
                 walker_id=None):
        super(NpcWalker, self).__init__(
            start_loc=start_loc,
            end_loc=end_loc,
            blueprint=blueprint,
            start_time=start_time
        )
        self.walker = None  # the actor object
        self.max_speed = max_speed
        self.behavior_type: int = behavior_type  # 0: walking, 1: stoped
        self.ai_controller: carla.WalkerAIController = None
        self.walker_id: str = walker_id
