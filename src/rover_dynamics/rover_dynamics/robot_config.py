from dataclasses import dataclass
from pathlib import Path
import yaml


@dataclass(frozen=True)
class BoxInertia:
    ixx: float
    iyy: float
    izz: float


@dataclass(frozen=True)
class BodyParameters:
    length: float
    width: float
    height: float
    mass: float

    @property
    def inertia(self):
        return BoxInertia(
            ixx=self.mass / 12.0 * (
                self.width**2 + self.height**2
            ),
            iyy=self.mass / 12.0 * (
                self.length**2 + self.height**2
            ),
            izz=self.mass / 12.0 * (
                self.length**2 + self.width**2
            ),
        )


@dataclass(frozen=True)
class WheelParameters:
    radius: float
    width: float
    mass: float
    x_offset: float
    y_offset: float
    z_offset: float

    @property
    def track_width(self):
        return 2.0 * self.y_offset

    @property
    def inertia_axis(self):
        return 0.5 * self.mass * self.radius**2

    @property
    def inertia_perpendicular(self):
        return (
            self.mass / 12.0
            * (3.0 * self.radius**2 + self.width**2)
        )


@dataclass(frozen=True)
class ArmLinkParameters:
    length: float
    width: float
    height: float
    mass: float

    @property
    def inertia(self):
        return BoxInertia(
            ixx=self.mass / 12.0 * (
                self.width**2 + self.height**2
            ),
            iyy=self.mass / 12.0 * (
                self.length**2 + self.height**2
            ),
            izz=self.mass / 12.0 * (
                self.length**2 + self.width**2
            ),
        )


@dataclass(frozen=True)
class ArmBaseParameters:
    radius: float
    height: float
    mass: float


@dataclass(frozen=True)
class CameraParameters:
    length: float
    width: float
    height: float
    mass: float


@dataclass(frozen=True)
class ActuatorParameters:
    torque_constant: float
    gear_ratio: float
    efficiency: float


@dataclass(frozen=True)
class ArmParameters:
    base: ArmBaseParameters
    link1: ArmLinkParameters
    link2: ArmLinkParameters
    wrist: ArmLinkParameters
    camera: CameraParameters


@dataclass(frozen=True)
class RoverParameters:
    body: BodyParameters
    wheels: WheelParameters
    arm: ArmParameters
    actuator: ActuatorParameters


def load_rover_parameters(config_path):
    config_path = Path(config_path)

    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    rover = config["rover"]

    body = BodyParameters(
        **rover["body"]
    )

    wheels = WheelParameters(
        **rover["wheels"]
    )

    arm = ArmParameters(
        base=ArmBaseParameters(
            **rover["arm"]["base"]
        ),
        link1=ArmLinkParameters(
            **rover["arm"]["link1"]
        ),
        link2=ArmLinkParameters(
            **rover["arm"]["link2"]
        ),
        wrist=ArmLinkParameters(
            **rover["arm"]["wrist"]
        ),
        camera=CameraParameters(
            **rover["arm"]["camera"]
        ),
    )

    actuator = ActuatorParameters(
        **rover["actuator"]
    )

    return RoverParameters(
        body=body,
        wheels=wheels,
        arm=arm,
        actuator=actuator,
    )