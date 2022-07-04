
class AGV:
    def __init__(self, aid: int, color, x, y, heading, agv_status, battery_level, charging_status, velocity, last_node_id, driving_status):
        self.aid = aid
        self.order = None
        self.x = x
        self.y = y
        self.heading = heading
        self.agv_status = agv_status
        self.battery_level = battery_level
        self.charging_status = charging_status
        self.velocity = velocity
        self.color = color
        self.last_node_id = last_node_id
        self.driving_status = driving_status

    def has_order(self):
        # Indicates if an AGV is currently executing an order
        return self.order is not None

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

    def update_last_nodeid(self, last_node_id, heading=None):
        if last_node_id == '':
            return
        self.last_node_id = last_node_id
        self.heading = heading
        if self.order is not None:
            self.order.update_last_node(last_node_id, (self.x, self.y))

            while self.order.extension_required(self.x, self.y):
                self.order.try_extension(self.x, self.y)

    def update_driving_status(self, driving_status, heading=None):
        self.driving_status = driving_status
        self.heading = heading
