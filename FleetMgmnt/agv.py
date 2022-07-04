
class AGV:
    def __init__(self, aid: int, color, x, y, heading, battery_level, charging_status, velocity, last_node_id, driving_status, connection_status):
        self.aid = aid
        self.x = x
        self.y = y
        self.heading = heading
        self.battery_level = battery_level
        self.charging_status = charging_status
        self.velocity = velocity
        self.color = color
        self.last_node_id = last_node_id
        self.driving_status = driving_status
        self.connection_status = connection_status

    def update_position(self, x, y, heading=None):
        self.x = x
        self.y = y
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

    def update_last_nodeid(self, last_node_id, heading=None):
        self.last_node_id = last_node_id
        self.heading = heading

    def update_driving_status(self, driving_status, heading=None):
        self.driving_status = driving_status
        self.heading = heading

    def update_connection_status(self, connection_status, heading=None):
        self.connection_status = connection_status
        self.heading = heading
