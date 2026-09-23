//! 高擎 (Hightorque) 电机寄存器表。
//!
//! 数据来源:高擎电机寄存器功能说明表(寄存器功能、电机运行模式、报错代码、
//! 一托多模式说明.xlsx sheet1)。地址范围 0x00–0x45,含保留项。
//!
//! 注意:该表只给出地址/名称/读写属性,并未给出每个寄存器的数据类型。
//! 读取时数据类型由 CAN 读命令的 cmd 半字节编码决定(见 `protocol::ReadCmd`),
//! 因此本表的 `ty` 字段仅在已从参考固件(`motor.c`)核实的情况下填充,
//! 其余为 `None`,调用方在读取时需显式指定类型。

use crate::protocol::RegisterType;

/// 寄存器元数据。
#[derive(Debug, Clone, Copy)]
pub struct RegisterInfo {
    /// 寄存器地址(0x00–0x45)。
    pub addr: u8,
    /// 寄存器名称。
    pub name: &'static str,
    /// 是否可写。
    pub writable: bool,
    /// 已知的数据类型;`None` 表示规范表未给出,读取时需显式指定。
    pub ty: Option<RegisterType>,
}

/// 完整寄存器表(按地址升序)。
pub const PARAMETER_TABLE: &[RegisterInfo] = &[
    RegisterInfo {
        addr: 0x00,
        name: "模式",
        writable: true,
        ty: None,
    },
    // 位置/速度/力矩:参考固件 motor.c 用 0x17 0x01 一次读 3 个 int16,类型已核实。
    RegisterInfo {
        addr: 0x01,
        name: "位置",
        writable: false,
        ty: Some(RegisterType::Int16),
    },
    RegisterInfo {
        addr: 0x02,
        name: "速度",
        writable: false,
        ty: Some(RegisterType::Int16),
    },
    RegisterInfo {
        addr: 0x03,
        name: "转矩",
        writable: false,
        ty: Some(RegisterType::Int16),
    },
    RegisterInfo {
        addr: 0x04,
        name: "Q 相电流",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x05,
        name: "D 相电流",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x06,
        name: "保留",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x0D,
        name: "电压",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x0E,
        name: "温度",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x0F,
        name: "错误代码",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x10,
        name: "PWM 相位 A",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x11,
        name: "PWM 相位 B",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x12,
        name: "PWM 相位 C",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x14,
        name: "电压相位 A",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x15,
        name: "电压相位 B",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x16,
        name: "电压相位 C",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x18,
        name: "电压 FOC 角度",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x19,
        name: "电压 FOC 电压",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x1A,
        name: "D 电压",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x1B,
        name: "Q 电压",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x1C,
        name: "Q 电流",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x1D,
        name: "D 电流",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x20,
        name: "位置指令",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x21,
        name: "速度命令",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x22,
        name: "前馈扭矩",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x23,
        name: "Kp 比例",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x24,
        name: "Kd 比例",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x25,
        name: "最大扭矩",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x26,
        name: "停止位置",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x27,
        name: "保留",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x28,
        name: "速度限制",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x29,
        name: "加速度限制",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x2B,
        name: "Kp",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x2C,
        name: "Kd",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x2D,
        name: "Ki",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x30,
        name: "比例扭矩",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x31,
        name: "积分扭矩",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x32,
        name: "微分扭矩",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x33,
        name: "前馈扭矩",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x34,
        name: "总控制扭矩",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x40,
        name: "下限",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x41,
        name: "上限",
        writable: true,
        ty: None,
    },
    RegisterInfo {
        addr: 0x42,
        name: "前馈扭矩(映射 0x22)",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x43,
        name: "Kp 比例(映射 0x23)",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x44,
        name: "Kd 比例(映射 0x24)",
        writable: false,
        ty: None,
    },
    RegisterInfo {
        addr: 0x45,
        name: "最大扭矩(映射 0x25)",
        writable: false,
        ty: None,
    },
];

/// 按地址查表。
pub fn parameter_info(addr: u8) -> Option<&'static RegisterInfo> {
    PARAMETER_TABLE.iter().find(|info| info.addr == addr)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parameter_table_has_no_duplicate_addresses() {
        let mut addrs: Vec<u8> = PARAMETER_TABLE.iter().map(|i| i.addr).collect();
        addrs.sort_unstable();
        let before = addrs.len();
        addrs.dedup();
        assert_eq!(addrs.len(), before, "duplicate register addresses present");
    }

    #[test]
    fn known_registers_resolved() {
        assert_eq!(parameter_info(0x01).unwrap().name, "位置");
        assert_eq!(parameter_info(0x0F).unwrap().name, "错误代码");
        assert_eq!(parameter_info(0x45).unwrap().name, "最大扭矩(映射 0x25)");
    }

    #[test]
    fn unknown_address_returns_none() {
        assert!(parameter_info(0x07).is_none());
        assert!(parameter_info(0x50).is_none());
    }

    #[test]
    fn pos_vel_tqe_types_are_int16() {
        assert_eq!(parameter_info(0x01).unwrap().ty, Some(RegisterType::Int16));
        assert_eq!(parameter_info(0x02).unwrap().ty, Some(RegisterType::Int16));
        assert_eq!(parameter_info(0x03).unwrap().ty, Some(RegisterType::Int16));
    }
}
