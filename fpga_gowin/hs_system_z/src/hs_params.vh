// ============================================================================
// hs_params.vh - sinh tu dong boi src/ocsvm_fpga/hwmodel/hs_export.py
// Khoi trich dac trung tren FPGA: dao ham -> Horn-Schunck -> luoi 4x4 -> nen log -> z-score
// Dac ta: claude/hs_fixed_point_spec.md ban 0.4 (muc 3-8). File ASCII.
// ============================================================================
`ifndef HS_PARAMS_VH
`define HS_PARAMS_VH

localparam integer UV_FORMAT_CHECK = 1306;   // 1306 = Q6.6 13 bit (ban 0.4)
localparam integer IMG_W       = 160;
localparam integer IMG_H       = 120;
// tang 1-2: dao ham x4 va he so a = Ix4/D16, b = Iy4/D16
localparam integer D4_W        = 11;      // Ix4, Iy4, It4 co dau
localparam integer D16_W       = 21;      // D16 = ALPHA2_16 + Ix4^2 + Iy4^2 khong dau
localparam integer ALPHA2_16   = 64;      // 16*alpha^2, alpha = 2
localparam integer COEF_W      = 15;      // a, b co dau
localparam integer COEF_FRAC   = 18;
// tang 3: Jacobi
localparam integer N_ITER      = 64;
localparam integer UV_W        = 13;      // u, v co dau, Q6.6
localparam integer UV_FRAC     = 6;
localparam integer AVG_MUL     = 10923;   // u_avg = ((2*canh + goc) * AVG_MUL + 2^(AVG_SHIFT-1)) >>> AVG_SHIFT
localparam integer AVG_SHIFT   = 17;
localparam integer N_W         = 25;      // N = Ix4*u_avg + Iy4*v_avg + (It4 << UV_FRAC)
// tang 4: |v| va trung binh o
localparam integer MAG_W       = 13;      // |v| khong dau
localparam integer MAG_ROUND   = 1;       // 1: |v| = (isqrt(4*(u^2+v^2)) + 1) >> 1 ; 0: isqrt(u^2+v^2)
localparam integer GRID_R      = 4;
localparam integer GRID_C      = 4;
localparam integer CELL_H      = 30;
localparam integer CELL_W      = 40;
localparam integer SUM_W       = 24;      // tong moi o co dau
localparam integer CELL_MUL    = 13981;    // mean = (SUM * CELL_MUL + 2^(CELL_SHIFT-1)) >>> CELL_SHIFT  -> Q5.10
localparam integer CELL_SHIFT  = 20;
// tang 5: nen log bang bang tra 2 vung
localparam integer LOG_A_BITS  = 10;      // vung A: |x| < LOG_A_LIMIT, dia chi = |x| >> LOG_A_STEP_SH
localparam integer LOG_A_STEP_SH = 1;
localparam integer LOG_A_LIMIT = 2048;
localparam integer LOG_B_STEP_SH = 6;    // vung B: dia chi = 2^LOG_A_BITS + ((|x| - LOG_A_LIMIT) >> LOG_B_STEP_SH)
localparam integer LOG_LUT_SIZE = 1504;
localparam integer LOG_OUT_W   = 12;      // khong dau Q1.10; ket qua = sign(x) * LUT
// tang 6: z = sat16(((g - MEAN) * INV + 2^(INV_FRAC-1)) >>> INV_FRAC)
localparam integer FEAT_W      = 16;      // dac trung, z: Q5.10 co dau
localparam integer FEAT_FRAC   = 10;
localparam integer DIM         = 48;
localparam integer INV_W       = 16;
localparam integer INV_FRAC    = 9;

`endif
