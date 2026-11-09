from dataclasses import dataclass
from enum import IntEnum


class Mode(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    FORCE_POS = 4
    ROBSTRIDE_POS_VEL_CSP = 5


@dataclass(frozen=True)
class MotorState:
    can_id: int
    arbitration_id: int
    status_code: int
    pos: float
    vel: float
    torq: float
    t_mos: float
    t_rotor: float
    # 错误码(0=无故障;与 status_code 分开,统一契约见 motor_abi state_ffi.rs):
    # HighTorque=表3 故障码;Damiao=状态枚举 0x08–0x0E 故障段;
    # Robstride=状态帧六故障位域(bit5..bit0,可并存);其余厂商恒 0。
    fault_code: int = 0


@dataclass(frozen=True)
class MotorStateSnapshot:
    state: MotorState
    feedback_age_s: float
    feedback_sequence: int
