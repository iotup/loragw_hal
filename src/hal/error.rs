#[derive(thiserror::Error, Debug)]
#[allow(non_camel_case_types)]
pub enum Error {
    #[error("LGW_HAL_ERROR")]
    LGW_HAL_ERROR,

    #[error("LGW_REG_ERROR")]
    LGW_REG_ERROR,

    #[error("LGW_COM_ERROR")]
    LGW_COM_ERROR,


    #[error("LGW_USB_ERROR")]
    LGW_USB_ERROR
}