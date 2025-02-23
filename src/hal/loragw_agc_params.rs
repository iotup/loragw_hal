

pub struct AgcGainParams {
    pub ana_min:u8,
    pub ana_max:u8,
    pub ana_thresh_l:u8,
    pub ana_thresh_h:u8,
    pub dec_attn_min:u8,
    pub dec_attn_max:u8,
    pub dec_thresh_l:u8,
    pub dec_thresh_h1:u8,
    pub dec_thresh_h2:u8,
    pub chan_attn_min:u8,
    pub chan_attn_max:u8,
    pub chan_thresh_l:u8,
    pub chan_thresh_h:u8,
    pub device_sel:u8,      /* sx1250 only */
    pub hp_max:u8,          /* sx1250 only */
    pub pa_duty_cycle:u8,    /* sx1250 only */
}

pub const AGC_PARAMS_SX1250:AgcGainParams = AgcGainParams {
    ana_min : 1,
    ana_max : 13,
    ana_thresh_l : 3,
    ana_thresh_h : 12,
    dec_attn_min : 4,
    dec_attn_max : 15,
    dec_thresh_l : 40,
    dec_thresh_h1 : 80,
    dec_thresh_h2 : 90,
    chan_attn_min : 4,
    chan_attn_max : 14,
    chan_thresh_l : 52,
    chan_thresh_h : 132,
    device_sel : 0,
    hp_max : 7,
    pa_duty_cycle : 4
};


pub  const  AGC_PARAMS_SX125X:AgcGainParams = AgcGainParams {
    ana_min : 0,
    ana_max : 9,
    ana_thresh_l : 16,
    ana_thresh_h : 35,
    dec_attn_min : 7,
    dec_attn_max : 11,
    dec_thresh_l : 45,
    dec_thresh_h1 : 100,
    dec_thresh_h2 : 115,
    chan_attn_min : 4,
    chan_attn_max : 14,
    chan_thresh_l : 52,
    chan_thresh_h : 132,
    device_sel : 0,
    hp_max : 0,
    pa_duty_cycle : 0
};