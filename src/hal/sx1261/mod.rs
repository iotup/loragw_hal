pub mod loragw_sx1261;
mod sx1261_def;
mod sx1261_pram;


/**
@struct lgw_spectral_scan_status_t
@brief Spectral Scan status
*/

#[allow(non_camel_case_types)]
#[derive(PartialEq,Clone, Copy)]
pub enum LgwSpectralScanStatus {
  LGW_SPECTRAL_SCAN_STATUS_NONE,
  LGW_SPECTRAL_SCAN_STATUS_ON_GOING,
  LGW_SPECTRAL_SCAN_STATUS_ABORTED,
  LGW_SPECTRAL_SCAN_STATUS_COMPLETED,
  LGW_SPECTRAL_SCAN_STATUS_UNKNOWN
}