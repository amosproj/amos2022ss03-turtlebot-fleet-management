class AGV:
    def __init__(self, aid: int, x, y, heading):
        self.aid = aid
        self.x = x
        self.y = y
        self.heading = heading

    def update_position(self, x, y, heading=None):
        self.x = x
        self.y = y
        self.heading = heading
