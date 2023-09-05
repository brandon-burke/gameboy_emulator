enum CpuState {
    Fetch,
    FetchPrefix,
    Execute { status: ExecuteStatus, machine_cycles: u8, temp_reg: u16 },
    InterruptHandle,
}

impl CpuState {
    fn get_temp_reg(&self) -> u16 {
        match self {
            CpuState::Fetch => panic!("Shouldn't call this function in the Fetch state"),
            CpuState::FetchPrefix => panic!("Shouldn't call this function in the FetchPrefix state"),
            CpuState::Execute { temp_reg, .. } => *temp_reg,
            CpuState::InterruptHandle => panic!("Shouldn't call this function in the InterruptHandle state"),
        }
    }
}

enum ExecuteStatus {
    Completed,
    Running,
    NotStarted,
}