use crate::{
    a320_headers::{
        ap_raw_output, athr_output, base_ecu_bus, base_elac_analog_outputs,
        base_elac_discrete_outputs, base_elac_out_bus, base_fac_analog_outputs, base_fac_bus,
        base_fac_discrete_outputs, base_fmgc_ap_fd_logic_outputs, base_fmgc_athr_outputs,
        base_fmgc_bus_inputs, base_fmgc_bus_outputs, base_fmgc_discrete_inputs,
        base_fmgc_discrete_outputs, base_fmgc_logic_outputs, base_fms_inputs,
        base_sec_analog_outputs, base_sec_discrete_outputs, base_sec_out_bus, AircraftSpecificData,
        BaseData, base_fcu_bus, base_fcu_discrete_outputs, base_fcu_logic_outputs, fcu_inputs, FmsVnavData, FmsVnavDiagData,
        athr_data, athr_data_computed, athr_input, athr_time, elac_inputs, fac_inputs, sec_inputs,
    },
    read_bytes,
};
use serde::Serialize;
use std::io::{prelude::*, Error};

pub const INTERFACE_VERSION: u64 = 3200006;
// Personal build: INTERFACE_VERSION plus the FCU model inputs and outputs at the end of each record
pub const INTERFACE_VERSION_FCU: u64 = 3209001;
// Personal build: INTERFACE_VERSION_FCU plus the FMS VNAV state at the end of each record
pub const INTERFACE_VERSION_FCU_VNAV: u64 = 3209002;
// Personal build: INTERFACE_VERSION_FCU_VNAV plus the descent/MCDU profile diagnostics
pub const INTERFACE_VERSION_FCU_VNAV_DIAG: u64 = 3209003;
// Personal build: INTERFACE_VERSION_FCU_VNAV_DIAG plus both FADECs' model inputs and outputs
pub const INTERFACE_VERSION_FADEC: u64 = 3209004;
// Personal build: INTERFACE_VERSION_FADEC plus the model inputs of ELAC 1, 2, SEC 1, 2, 3 and FAC 1, 2
pub const INTERFACE_VERSION_FBW: u64 = 3209005;

// A single FDR record
#[derive(Serialize, Default)]
pub struct FdrData {
    base: BaseData,
    specific: AircraftSpecificData,
    elac_1: ElacData,
    elac_2: ElacData,
    sec_1: SecData,
    sec_2: SecData,
    sec_3: SecData,
    fac_1: FacData,
    fac_2: FacData,
    fmgc_1: FmgcData,
    fadec_1: FadecData,
}

// A single FDR record of INTERFACE_VERSION_FCU
#[derive(Serialize, Default)]
pub struct FdrDataFcu {
    base: BaseData,
    specific: AircraftSpecificData,
    elac_1: ElacData,
    elac_2: ElacData,
    sec_1: SecData,
    sec_2: SecData,
    sec_3: SecData,
    fac_1: FacData,
    fac_2: FacData,
    fmgc_1: FmgcData,
    fadec_1: FadecData,
    fcu: FcuData,
}

// A single FDR record of INTERFACE_VERSION_FCU_VNAV
#[derive(Serialize, Default)]
pub struct FdrDataFcuVnav {
    base: BaseData,
    specific: AircraftSpecificData,
    elac_1: ElacData,
    elac_2: ElacData,
    sec_1: SecData,
    sec_2: SecData,
    sec_3: SecData,
    fac_1: FacData,
    fac_2: FacData,
    fmgc_1: FmgcData,
    fadec_1: FadecData,
    fcu: FcuData,
    fms_vnav: FmsVnavData,
}

// A single FDR record of INTERFACE_VERSION_FCU_VNAV_DIAG
#[derive(Serialize, Default)]
pub struct FdrDataFcuVnavDiag {
    base: BaseData,
    specific: AircraftSpecificData,
    elac_1: ElacData,
    elac_2: ElacData,
    sec_1: SecData,
    sec_2: SecData,
    sec_3: SecData,
    fac_1: FacData,
    fac_2: FacData,
    fmgc_1: FmgcData,
    fadec_1: FadecData,
    fcu: FcuData,
    fms_vnav: FmsVnavData,
    fms_vnav_diag: FmsVnavDiagData,
}

// A single FDR record of INTERFACE_VERSION_FADEC
#[derive(Serialize, Default)]
pub struct FdrDataFadec {
    base: BaseData,
    specific: AircraftSpecificData,
    elac_1: ElacData,
    elac_2: ElacData,
    sec_1: SecData,
    sec_2: SecData,
    sec_3: SecData,
    fac_1: FacData,
    fac_2: FacData,
    fmgc_1: FmgcData,
    fadec_1: FadecData,
    fcu: FcuData,
    fms_vnav: FmsVnavData,
    fms_vnav_diag: FmsVnavDiagData,
    fadec_1_model: FadecModelData,
    fadec_2_model: FadecModelData,
}

// A single FDR record of INTERFACE_VERSION_FBW
#[derive(Serialize, Default)]
pub struct FdrDataFbw {
    base: BaseData,
    specific: AircraftSpecificData,
    elac_1: ElacData,
    elac_2: ElacData,
    sec_1: SecData,
    sec_2: SecData,
    sec_3: SecData,
    fac_1: FacData,
    fac_2: FacData,
    fmgc_1: FmgcData,
    fadec_1: FadecData,
    fcu: FcuData,
    fms_vnav: FmsVnavData,
    fms_vnav_diag: FmsVnavDiagData,
    fadec_1_model: FadecModelData,
    fadec_2_model: FadecModelData,
    // the model inputs of each computer's last step (a FAC that did not step repeats its previous block)
    elac_1_inputs: elac_inputs,
    elac_2_inputs: elac_inputs,
    sec_1_inputs: sec_inputs,
    sec_2_inputs: sec_inputs,
    sec_3_inputs: sec_inputs,
    fac_1_inputs: fac_inputs,
    fac_2_inputs: fac_inputs,
}

pub fn read_record_fbw(reader: &mut impl Read) -> Result<FdrDataFbw, Error> {
    let r = read_record_fadec(reader)?;
    Ok(FdrDataFbw {
        base: r.base,
        specific: r.specific,
        elac_1: r.elac_1,
        elac_2: r.elac_2,
        sec_1: r.sec_1,
        sec_2: r.sec_2,
        sec_3: r.sec_3,
        fac_1: r.fac_1,
        fac_2: r.fac_2,
        fmgc_1: r.fmgc_1,
        fadec_1: r.fadec_1,
        fcu: r.fcu,
        fms_vnav: r.fms_vnav,
        fms_vnav_diag: r.fms_vnav_diag,
        fadec_1_model: r.fadec_1_model,
        fadec_2_model: r.fadec_2_model,
        elac_1_inputs: read_bytes::<elac_inputs>(reader)?,
        elac_2_inputs: read_bytes::<elac_inputs>(reader)?,
        sec_1_inputs: read_bytes::<sec_inputs>(reader)?,
        sec_2_inputs: read_bytes::<sec_inputs>(reader)?,
        sec_3_inputs: read_bytes::<sec_inputs>(reader)?,
        fac_1_inputs: read_bytes::<fac_inputs>(reader)?,
        fac_2_inputs: read_bytes::<fac_inputs>(reader)?,
    })
}

// One FADEC's model inputs (time, data, input, fcu_input) and outputs
#[derive(Serialize, Default)]
struct FadecModelData {
    time: athr_time,
    data: athr_data,
    input: athr_input,
    fcu_input: base_fcu_bus,
    data_computed: athr_data_computed,
    output: athr_output,
    bus_outputs: base_ecu_bus,
}

pub fn read_record_fadec(reader: &mut impl Read) -> Result<FdrDataFadec, Error> {
    let r = read_record_fcu_vnav_diag(reader)?;
    Ok(FdrDataFadec {
        base: r.base,
        specific: r.specific,
        elac_1: r.elac_1,
        elac_2: r.elac_2,
        sec_1: r.sec_1,
        sec_2: r.sec_2,
        sec_3: r.sec_3,
        fac_1: r.fac_1,
        fac_2: r.fac_2,
        fmgc_1: r.fmgc_1,
        fadec_1: r.fadec_1,
        fcu: r.fcu,
        fms_vnav: r.fms_vnav,
        fms_vnav_diag: r.fms_vnav_diag,
        fadec_1_model: read_fadec_model(reader)?,
        fadec_2_model: read_fadec_model(reader)?,
    })
}

fn read_fadec_model(reader: &mut impl Read) -> Result<FadecModelData, Error> {
    Ok(FadecModelData {
        time: read_bytes::<athr_time>(reader)?,
        data: read_bytes::<athr_data>(reader)?,
        input: read_bytes::<athr_input>(reader)?,
        fcu_input: read_bytes::<base_fcu_bus>(reader)?,
        data_computed: read_bytes::<athr_data_computed>(reader)?,
        output: read_bytes::<athr_output>(reader)?,
        bus_outputs: read_bytes::<base_ecu_bus>(reader)?,
    })
}

pub fn read_record_fcu_vnav_diag(reader: &mut impl Read) -> Result<FdrDataFcuVnavDiag, Error> {
    let r = read_record_fcu_vnav(reader)?;
    Ok(FdrDataFcuVnavDiag {
        base: r.base,
        specific: r.specific,
        elac_1: r.elac_1,
        elac_2: r.elac_2,
        sec_1: r.sec_1,
        sec_2: r.sec_2,
        sec_3: r.sec_3,
        fac_1: r.fac_1,
        fac_2: r.fac_2,
        fmgc_1: r.fmgc_1,
        fadec_1: r.fadec_1,
        fcu: r.fcu,
        fms_vnav: r.fms_vnav,
        fms_vnav_diag: read_bytes::<FmsVnavDiagData>(reader)?,
    })
}

#[derive(Serialize, Default)]
struct FcuData {
    inputs: fcu_inputs,
    logic: base_fcu_logic_outputs,
    discrete_outputs: base_fcu_discrete_outputs,
    bus_outputs: base_fcu_bus,
}

#[derive(Serialize, Default)]
struct ElacData {
    bus_outputs: base_elac_out_bus,
    discrete_outputs: base_elac_discrete_outputs,
    analog_outputs: base_elac_analog_outputs,
}

#[derive(Serialize, Default)]
struct SecData {
    bus_outputs: base_sec_out_bus,
    discrete_outputs: base_sec_discrete_outputs,
    analog_outputs: base_sec_analog_outputs,
}

#[derive(Serialize, Default)]
struct FacData {
    bus_outputs: base_fac_bus,
    discrete_outputs: base_fac_discrete_outputs,
    analog_outputs: base_fac_analog_outputs,
}

#[derive(Serialize, Default)]
struct FmgcData {
    logic: base_fmgc_logic_outputs,
    ap_fd_logic: base_fmgc_ap_fd_logic_outputs,
    ap_fd_outer_loops: ap_raw_output,
    athr: base_fmgc_athr_outputs,
    discrete_outputs: base_fmgc_discrete_outputs,
    bus_outputs: base_fmgc_bus_outputs,
    bus_inputs: base_fmgc_bus_inputs,
    discrete_inputs: base_fmgc_discrete_inputs,
    fms_inputs: base_fms_inputs,
}

#[derive(Serialize, Default)]
struct FadecData {
    bus_outputs: base_ecu_bus,
    outputs: athr_output,
}

// These are helper functions to read in a whole FDR record.
pub fn read_record(reader: &mut impl Read) -> Result<FdrData, Error> {
    Ok(FdrData {
        base: read_bytes::<BaseData>(reader)?,
        specific: read_bytes::<AircraftSpecificData>(reader)?,
        elac_1: read_elac(reader)?,
        elac_2: read_elac(reader)?,
        sec_1: read_sec(reader)?,
        sec_2: read_sec(reader)?,
        sec_3: read_sec(reader)?,
        fac_1: read_fac(reader)?,
        fac_2: read_fac(reader)?,
        fmgc_1: read_fmgc(reader)?,
        fadec_1: read_fadec(reader)?,
    })
}

pub fn read_record_fcu_vnav(reader: &mut impl Read) -> Result<FdrDataFcuVnav, Error> {
    let r = read_record_fcu(reader)?;
    Ok(FdrDataFcuVnav {
        base: r.base,
        specific: r.specific,
        elac_1: r.elac_1,
        elac_2: r.elac_2,
        sec_1: r.sec_1,
        sec_2: r.sec_2,
        sec_3: r.sec_3,
        fac_1: r.fac_1,
        fac_2: r.fac_2,
        fmgc_1: r.fmgc_1,
        fadec_1: r.fadec_1,
        fcu: r.fcu,
        fms_vnav: read_bytes::<FmsVnavData>(reader)?,
    })
}

pub fn read_record_fcu(reader: &mut impl Read) -> Result<FdrDataFcu, Error> {
    let r = read_record(reader)?;
    Ok(FdrDataFcu {
        base: r.base,
        specific: r.specific,
        elac_1: r.elac_1,
        elac_2: r.elac_2,
        sec_1: r.sec_1,
        sec_2: r.sec_2,
        sec_3: r.sec_3,
        fac_1: r.fac_1,
        fac_2: r.fac_2,
        fmgc_1: r.fmgc_1,
        fadec_1: r.fadec_1,
        fcu: FcuData {
            inputs: read_bytes::<fcu_inputs>(reader)?,
            logic: read_bytes::<base_fcu_logic_outputs>(reader)?,
            discrete_outputs: read_bytes::<base_fcu_discrete_outputs>(reader)?,
            bus_outputs: read_bytes::<base_fcu_bus>(reader)?,
        },
    })
}

fn read_elac(reader: &mut impl Read) -> Result<ElacData, Error> {
    Ok(ElacData {
        bus_outputs: read_bytes::<base_elac_out_bus>(reader)?,
        discrete_outputs: read_bytes::<base_elac_discrete_outputs>(reader)?,
        analog_outputs: read_bytes::<base_elac_analog_outputs>(reader)?,
    })
}

fn read_sec(reader: &mut impl Read) -> Result<SecData, Error> {
    Ok(SecData {
        bus_outputs: read_bytes::<base_sec_out_bus>(reader)?,
        discrete_outputs: read_bytes::<base_sec_discrete_outputs>(reader)?,
        analog_outputs: read_bytes::<base_sec_analog_outputs>(reader)?,
    })
}

fn read_fac(reader: &mut impl Read) -> Result<FacData, Error> {
    Ok(FacData {
        bus_outputs: read_bytes::<base_fac_bus>(reader)?,
        discrete_outputs: read_bytes::<base_fac_discrete_outputs>(reader)?,
        analog_outputs: read_bytes::<base_fac_analog_outputs>(reader)?,
    })
}

fn read_fmgc(reader: &mut impl Read) -> Result<FmgcData, Error> {
    Ok(FmgcData {
        logic: read_bytes::<base_fmgc_logic_outputs>(reader)?,
        ap_fd_logic: read_bytes::<base_fmgc_ap_fd_logic_outputs>(reader)?,
        ap_fd_outer_loops: read_bytes::<ap_raw_output>(reader)?,
        athr: read_bytes::<base_fmgc_athr_outputs>(reader)?,
        discrete_outputs: read_bytes::<base_fmgc_discrete_outputs>(reader)?,
        bus_outputs: read_bytes::<base_fmgc_bus_outputs>(reader)?,
        bus_inputs: read_bytes::<base_fmgc_bus_inputs>(reader)?,
        discrete_inputs: read_bytes::<base_fmgc_discrete_inputs>(reader)?,
        fms_inputs: read_bytes::<base_fms_inputs>(reader)?,
    })
}

fn read_fadec(reader: &mut impl Read) -> Result<FadecData, Error> {
    Ok(FadecData {
        bus_outputs: read_bytes::<base_ecu_bus>(reader)?,
        outputs: read_bytes::<athr_output>(reader)?,
    })
}
