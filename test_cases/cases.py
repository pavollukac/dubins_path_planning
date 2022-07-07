from math import pi


class TestCase:
    """ Provide some test cases for a 10x10 map. """

    def __init__(self):

        # self.start_pos = [4.6, 2.4, 0]
        # self.end_pos = [1.6, 8, -pi/2]

        # self.start_pos2 = [4, 4, 0]
        # self.end_pos2 = [4, 8, 1.2*pi]

        # self.obs = [
        #     [2, 3, 6, 0.1],
        #     [2, 3, 0.1, 1.5],
        #     [4.3, 0, 0.1, 1.8],
        #     [6.5, 1.5, 0.1, 1.5],
        #     [0, 6, 3.5, 0.1],
        #     [5, 6, 5, 0.1]
        # ]
        
        self.start_pos = [0.15, .15, +pi/2]
        self.end_pos = [49, 39, 0]
        self.end_pos2 = [15, 11, 0]
        self.start_pos3 = [48.5, .3, pi/8]
        self.end_pos3 = [49.5, 5, +pi/2]

        self.obs = [
            [ 1.5, 1.5, 48, 8],
            [ 1.5, 12, 48, 26],
            [17,1.5, 32, 37]            
        ]