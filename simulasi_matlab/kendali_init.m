function st = kendali_init(cfg, useFuzzy)
%KENDALI_INIT  Keadaan awal kendali = switchToOtomatis() + reset riwayat filter (runBaru).
st.state = 1;            % JELAJAH
st.useFuzzy = useFuzzy;
st.run = 1;
st.hL = [999 999 999]; st.hR = [999 999 999]; st.hi = 0;
st.fL = -1; st.fR = -1; st.nL = 0; st.nR = 0;
st.hF = [999 999 999]; st.fi = 0;
st.lastFront = 999; st.nHilang = 0;
st.fRef = 999; st.tRef = 0;
st.turnStartMs = 0; st.turnDir = 0; st.fase = 0; st.faseStart = 0; st.mundurCount = 0;
st.lihatN = 0; st.lihatL = 0; st.lihatR = 0; st.lihatF = 999;
st.jedaStepper = false;
st.integral = 0; st.errorSeb = 0; st.error = 0; st.deltaError = 0;
st.Kp = cfg.k.Kp0; st.Ki = cfg.k.Ki0; st.Kd = cfg.k.Kd0;
st.P = 0; st.I = 0; st.D = 0; st.u = 0;
st.tgtAngle = 0; st.motorSpeed = 0; st.pumpOn = false; st.waterLow = false;
st.lastHalf = cfg.k.SETPOINT_KIRI_CM;
end
