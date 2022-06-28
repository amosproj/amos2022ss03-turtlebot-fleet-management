
class AGV:
    def __init__(self, aid: int, color, x, y, heading, agv_status, battery_level, charging_status, velocity):
        self.aid = aid
        self.x = x
        self.y = y
        self.heading = heading
        self.agv_status = agv_status
        self.battery_level = battery_level
        self.charging_status = charging_status
        self.velocity = velocity
        self.color = color

    def update_position(self, x, y, heading=None):
        self.x = x
        self.y = y
        self.heading = heading

    def update_status(self, status, heading=None):
        self.agv_status = status
        self.heading = heading

    def update_battery_level(self, battery, heading=None):
        self.battery_level = battery
        self.heading = heading

    def update_charging_status(self, charging_status, heading=None):
        self.charging_status = charging_status
        self.heading = heading

    def update_velocity(self, velocity, heading=None):
        self.velocity = velocity
        self.heading = heading

