import Tkinter as tk

class QLearningVisualization(object):

    def __init__(self, mission_type, q_table, actions):
        self.actions = actions
        self.q_table = q_table
        self.scale = 40
        self.mission_type = mission_type
        self.world_x = 0
        self.world_y = 0
        # Set world dimensions for visualization
        if self.mission_type == "small":
            self.world_x = 10
            self.world_y = 10
        elif self.mission_type == "medium":
            self.world_x = 20
            self.world_y = 20
        else:
            self.world_x = 40
            self.world_y = 40

        self.init_canvas()

    def init_canvas(self,):
        self.root = tk.Tk()
        self.root.wm_title("Q-table")
        self.canvas = tk.Canvas(self.root, width=self.world_x * self.scale, height=self.world_y * self.scale,
                                borderwidth=0, highlightthickness=0, bg="black")
        self.canvas.grid()
        self.root.update()

    def drawQ(self, curr_x=None, curr_y=None):
        if self.canvas is None or self.root is None:
            return
        self.canvas.delete("all")
        action_inset = 0.1
        action_radius = 0.1
        curr_radius = 0.2
        action_positions = [ ( 0.5, 1-action_inset ), ( 0.5, action_inset ), ( 1-action_inset, 0.5 ), ( action_inset, 0.5 ) ]
        # (NSWE to match action order)
        min_value = -20
        max_value = 20
        for x in range(self.world_x):
            for y in range(self.world_y):
                s = "S_%d_%d" % (x,y)
                self.canvas.create_rectangle( (self.world_x-1-x)*self.scale, (self.world_y-1-y)*self.scale, (self.world_x-1-x+1)*self.scale, (self.world_y-1-y+1)*self.scale, outline="#fff", fill="#000")
                for a_index, action in enumerate(self.actions):
                    if not (s, action) in self.q_table:
                        continue
                    value = self.q_table[s, action]
                    color = 255 * ( value - min_value ) / ( max_value - min_value ) # map value to 0-255
                    color = max( min( color, 255 ), 0 ) # ensure within [0,255]
                    color_string = '#%02x%02x%02x' % (255-color, color, 0)
                    self.canvas.create_oval( (self.world_x - 1 - x + action_positions[a_index][0] - action_radius ) *self.scale,
                                             (self.world_y - 1 - y + action_positions[a_index][1] - action_radius ) *self.scale,
                                             (self.world_x - 1 - x + action_positions[a_index][0] + action_radius ) *self.scale,
                                             (self.world_y - 1 - y + action_positions[a_index][1] + action_radius ) *self.scale,
                                             outline=color_string, fill=color_string )
        if curr_x is not None and curr_y is not None:
            self.canvas.create_oval( (self.world_x - 1 - curr_x + 0.5 - curr_radius ) * self.scale,
                                     (self.world_y - 1 - curr_y + 0.5 - curr_radius ) * self.scale,
                                     (self.world_x - 1 - curr_x + 0.5 + curr_radius ) * self.scale,
                                     (self.world_y - 1 - curr_y + 0.5 + curr_radius ) * self.scale,
                                     outline="#fff", fill="#fff" )
        self.root.update()
