from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import PowerDistribution, DriverStation
# from helpers.andymark.AM_CAN_Color_Sensor import AMCANColorSensor
# from wpilib import Color, Color8Bit


class UtilSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self.auto_start_time = 0

        self.scoring_location = 0

        self._inst = NetworkTableInstance.getDefault()
        self._table = self._inst.getTable("PyTimer")

        self._received_game_data = False
        self._output_team_color = False

        # self._color_sensor = AMCANColorSensor(60)

        self._hub_activity = {
            "red_winner_red_alliance": [
                [(2 * 60) + 20 + 3, (2 * 60) + 10 + 2],
                [(1 * 60) + 45 + 3, (1 * 60) + 20 + 2],
                [55 + 3, 0 - 5]
            ],
            "blue_winner_red_alliance": [
                [(2 * 60) + 20 + 3, (1 * 60) + 45 + 2],
                [(1 * 60) + 20 + 3, 55 + 2],
                [30 + 3, 0 - 5]
            ],
            "red_winner_blue_alliance": [
                [(2 * 60) + 20 + 3, (1 * 60) + 45 + 2],
                [(1 * 60) + 20 + 3, 55 + 2],
                [30 + 3, 0 - 5]
            ],
            "blue_winner_blue_alliance": [
                [(2 * 60) + 20 + 3, (2 * 60) + 10 + 2],
                [(1 * 60) + 45 + 3, (1 * 60) + 20 + 2],
                [55 + 3, 0 - 5]
            ],
        }

        # FORMAT: X, Y, ANGLE, LOCATION NAME
        self.scoring_sides_red = [
            [13.766, 4.031, [
                [13.280, 4.107, 180.001, "Red A Flipped"],
                [13.247, 4.439, 180.001, "Red B Flipped"],
                [13.258, 3.610, 180.001, "Red A Normal"],
                [13.245, 3.935, 180.001, "Red B Normal"],
            ], 7],
            [13.404, 4.609, [
                [13.099, 4.248, 240, "Red C Flipped"],
                [12.791, 4.385, 240, "Red D Flipped"],
                [13.520, 3.985, 240, "Red C Normal"],
                [13.231, 4.146, 240, "Red D Normal"],
            ], 8],
            [12.729, 4.585, [
                [12.905, 4.127, 120, "Red E Flipped"],
                [12.600, 3.996, 120, "Red F Flipped"],
                [13.332, 4.394, 120, "Red E Normal"],
                [13.024, 4.264, 120, "Red F Normal"],
            ], 9],
            [12.384, 4.015, [
                [12.875, 3.936, 0.001, "Red G Flipped"],
                [12.886, 3.603, 0.001, "Red H Flipped"],
                [12.812, 4.445, 0.001, "Red G Normal"],
                [12.832, 4.109, 0.001, "Red H Normal"],
            ], 10],
            [12.729, 3.412, [
                [13.032, 3.792, 60, "Red I Flipped"],
                [13.317, 3.641, 60, "Red J Flipped"],
                [12.594, 4.046, 60, "Red I Normal"],
                [12.889, 3.898, 60, "Red J Normal"],
            ], 11],
            [13.388, 3.482, [
                [13.227, 3.909, 300, "Red K Flipped"],
                [13.528, 4.053, 300, "Red L Flipped"],
                [12.813, 3.628, 300, "Red K Normal"],
                [13.056, 3.797, 300, "Red L Normal"],
            ], 6]
        ]
        self.scoring_sides_blue = [
            [17.513 - 13.766, 8.021 - 4.031, [
                [17.513 - 13.280, 8.021 - 4.107, 0.001, "Blue A Flipped"],
                [17.513 - 13.247, 8.021 - 4.439, 0.001, "Blue B Flipped"],
                [17.513 - 13.258, 8.021 - 3.610, 0.001, "Blue A Normal"],
                [17.513 - 13.245, 8.021 - 3.935, 0.001, "Blue B Normal"],
            ], 18],
            [17.513 - 13.404, 8.021 - 4.609, [
                [17.513 - 13.099, 8.021 - 4.248, 60, "Blue C Flipped"],
                [17.513 - 12.791, 8.021 - 4.385, 60, "Blue D Flipped"],
                [17.513 - 13.520, 8.021 - 3.985, 60, "Blue C Normal"],
                [17.513 - 13.231, 8.021 - 4.146, 60, "Blue D Normal"],
            ], 17],
            [17.513 - 12.729, 8.021 - 4.585, [
                [17.513 - 12.905, 8.021 - 4.127, 300, "Blue E Flipped"],
                [17.513 - 12.600, 8.021 - 3.996, 300, "Blue F Flipped"],
                [17.513 - 13.332, 8.021 - 4.394, 300, "Blue E Normal"],
                [17.513 - 13.024, 8.021 - 4.264, 300, "Blue F Normal"],
            ], 22],
            [17.513 - 12.384, 8.021 - 4.015, [
                [17.513 - 12.875, 8.021 - 3.936, 180.001, "Blue G Flipped"],
                [17.513 - 12.848, 8.021 - 3.607, 180.001, "Blue H Flipped"],
                [17.513 - 12.812, 8.021 - 4.445, 180.001, "Blue G Normal"],
                [17.513 - 12.832, 8.021 - 4.109, 180.001, "Blue H Normal"],
            ], 21],
            [17.513 - 12.729, 8.021 - 3.412, [
                [17.513 - 13.032, 8.021 - 3.792, 240, "Blue I Flipped"],
                [17.513 - 13.317, 8.021 - 3.641, 240, "Blue J Flipped"],
                [17.513 - 12.594, 8.021 - 4.046, 240, "Blue I Normal"],
                [17.513 - 12.889, 8.021 - 3.898, 240, "Blue J Normal"],
            ], 20],
            [17.513 - 13.388, 8.021 - 3.482, [
                [17.513 - 13.227, 8.021 - 3.909, 120, "Blue K Flipped"],
                [17.513 - 13.528, 8.021 - 4.053, 120, "Blue L Flipped"],
                [17.513 - 12.813, 8.021 - 3.628, 120, "Blue K Normal"],
                [17.513 - 13.056, 8.021 - 3.797, 120, "Blue L Normal"],
            ], 19]
        ]

        self.feeder_sides_red = [
            [17.5, 8.5, 55],
            [17.5, 0, 125]
        ]
        self.feeder_sides_blue = [
            [0.683, 7.245, 305],
            [0.683, 0.758, 235]
        ]


    def toggle_channel(self, on: bool) -> None:
        self.pdh.setSwitchableChannel(on)

    def get_game_data_received(self) -> bool:
        return self._received_game_data

    def get_alliance_winner(self) -> str:
        if self._table.getString("Auto Winner", "N") == "R":
            return "red"
        else:
            return "blue"

    def get_hub_active(self) -> bool:
        if not self._output_team_color or not self._received_game_data:
            return True
        else:
            hub_active = False
            if self._table.getString("Auto Winner", "N") == "R":
                if self._table.getString("Team Color", "B") == "R":
                    for x in self._hub_activity["red_winner_red_alliance"]:
                        if x[1] <= DriverStation.getMatchTime() <= x[0]:
                            hub_active = True
                else:
                    for x in self._hub_activity["red_winner_blue_alliance"]:
                        if x[1] <= DriverStation.getMatchTime() <= x[0]:
                            hub_active = True
            else:
                if self._table.getString("Team Color", "B") == "R":
                    for x in self._hub_activity["blue_winner_red_alliance"]:
                        if x[1] <= DriverStation.getMatchTime() <= x[0]:
                            hub_active = True
                else:
                    for x in self._hub_activity["blue_winner_blue_alliance"]:
                        if x[1] <= DriverStation.getMatchTime() <= x[0]:
                            hub_active = True
            return hub_active


    def get_shift_time_remaining(self) -> int:
        current_time = DriverStation.getMatchTime()
        if 130 < current_time <= 140:
            return current_time - 130
        elif 105 < current_time <= 130:
            return current_time - 105
        elif 80 < current_time <= 105:
            return current_time - 80
        elif 55 < current_time <= 80:
            return current_time - 55
        elif 30 < current_time <= 55:
            return current_time - 30
        else:
            return current_time

    def get_inactive_warning(self, time: float) -> bool:
        if self.get_hub_active() and self.get_shift_time_remaining() <= time and self.get_shift_time_remaining() != -1:
            return True
        else:
            return False


    def periodic(self) -> None:
        self._table.putNumber("Match Timer", DriverStation.getMatchTime())
        self._table.putBoolean("Active Shift?", self.get_hub_active())
        self._table.putNumber("Time Remaining in Shift", self.get_shift_time_remaining())
        if not self._received_game_data:
            if DriverStation.isTeleop():
                _fms_game_data = str(DriverStation.getGameSpecificMessage())
                if _fms_game_data == "B" or _fms_game_data == "R":
                    self._received_game_data = True
                    self._table.putString("Auto Winner", _fms_game_data)
        if not self._output_team_color:
            if DriverStation.isDSAttached():
                self._output_team_color = True
                if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                    self._table.putString("Team Color", "R")
                else:
                    self._table.putString("Team Color", "B")

        # self._table.putNumber("Proximity", self._color_sensor.get_data().proximity)
        # self._table.putValue("Color", Color(self._color_sensor.get_data().red,
        #       self._color_sensor.get_data().green,
        #       self._color_sensor.get_data().blue
        #       ).hexString())
