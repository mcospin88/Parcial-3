// =============================================================
// Basys3 Elevator FSM - top.sv (SystemVerilog, sin BUFG)
// - Usa directamente el puerto clk (W5) y deja que Vivado inserte BUFG
// - sw[0]=modo display; sw[3:1]=destino (0..7); sw[4]=alarma; sw[5]=sensores manuales
// - sw[6]=obstáculo; sw[7]=door_closed_ok; sw[8]=door_open_ok; sw[9]=timer_done (si manual)
// - led[15:0] para diagnóstico
// =============================================================

module top(
    input  wire        clk,          // 100 MHz (W5)
    input  wire        btnC,         // Reset síncrono
    input  wire        btnU,         // Llamada exterior: Subir (▲)
    input  wire        btnD,         // Llamada exterior: Bajar (▼)
    input  wire        btnL,         // Abrir puerta (◄ ►)
    input  wire        btnR,         // Cerrar puerta (► ◄)
    input  wire [15:0] sw,           // ver descripción arriba
    output wire [15:0] led,
    output wire [3:0]  an,           // ánodos activos-bajo
    output wire [7:0]  seg           // {a..g,dp} activos-bajo
);

    // -------------------- Reset -------------------------------
    wire rst = btnC;

    // -------------------- Enables -----------------------------
    wire tick_1hz, tick_1khz;
    clk_enable uce(.clk(clk), .rst(rst), .tick_1hz(tick_1hz), .tick_1khz(tick_1khz));

    // -------------------- Debounce & one-shot -----------------
    wire bU_d, bD_d, bL_d, bR_d;
    debouncer dbU(.clk(clk), .rst(rst), .noisy(btnU), .clean(bU_d));
    debouncer dbD(.clk(clk), .rst(rst), .noisy(btnD), .clean(bD_d));
    debouncer dbL(.clk(clk), .rst(rst), .noisy(btnL), .clean(bL_d));
    debouncer dbR(.clk(clk), .rst(rst), .noisy(btnR), .clean(bR_d));

    wire call_up_p, call_down_p, door_open_p, door_close_p;
    one_shot osU(.clk(clk), .rst(rst), .in(bU_d), .pulse(call_up_p));
    one_shot osD(.clk(clk), .rst(rst), .in(bD_d), .pulse(call_down_p));
    one_shot osL(.clk(clk), .rst(rst), .in(bL_d), .pulse(door_open_p));
    one_shot osR(.clk(clk), .rst(rst), .in(bR_d), .pulse(door_close_p));

    // -------------------- Entradas de usuario -----------------
    wire [2:0] dest_floor    = sw[3:1];
    wire       alarm_on      = sw[4];
    wire       manual_sensors= sw[5];

    // -------------------- Sensores (manual/auto) --------------
    wire sensor_obstacle_w;
    wire sensor_door_closed_ok_w;
    wire sensor_door_open_ok_w;
    wire door_timer_done_w;

    // Auto (desde control de puertas)
    wire door_m_open_cmd, door_m_close_cmd;
    wire door_auto_open_ok, door_auto_closed_ok;
    wire door_timer_active, door_timer_done_auto;

    assign sensor_obstacle_w       = manual_sensors ? sw[6] : 1'b0;
    assign sensor_door_closed_ok_w = manual_sensors ? sw[7] : door_auto_closed_ok;
    assign sensor_door_open_ok_w   = manual_sensors ? sw[8] : door_auto_open_ok;
    assign door_timer_done_w       = manual_sensors ? sw[9] : door_timer_done_auto;

    // -------------------- Parámetros demo ---------------------
    localparam int N_FLOORS     = 8; // 0..7
    localparam int FLOOR_TIME_S = 2; // seg por piso
    localparam int OPEN_TIME_S  = 3; // puertas abiertas
    localparam int DOOR_ACT_S   = 1; // acción puerta

    // -------------------- Estado cabina -----------------------
    logic [2:0] cur_floor;
    logic [2:0] target_floor;
    logic       motor_up, motor_down;
    wire        motor_stop = ~(motor_up | motor_down);

    logic [3:0] floor_tick_cnt;

    always_ff @(posedge clk) begin
        if (rst) begin
            cur_floor      <= 3'd0;
            floor_tick_cnt <= 0;
        end else begin
            if (motor_up ^ motor_down) begin
                if (tick_1hz) begin
                    if (floor_tick_cnt == FLOOR_TIME_S-1) begin
                        floor_tick_cnt <= 0;
                        if (motor_up   && cur_floor < N_FLOORS-1) cur_floor <= cur_floor + 1'b1;
                        if (motor_down && cur_floor > 0)         cur_floor <= cur_floor - 1'b1;
                    end else begin
                        floor_tick_cnt <= floor_tick_cnt + 1'b1;
                    end
                end
            end else begin
                floor_tick_cnt <= 0;
            end
        end
    end

    // -------------------- Control puertas + timer -------------
    logic [3:0] door_act_cnt;
    always_ff @(posedge clk) begin
        if (rst) door_act_cnt <= 0;
        else if (door_m_open_cmd || door_m_close_cmd) begin
            if (tick_1hz) begin
                if (door_act_cnt == DOOR_ACT_S) door_act_cnt <= DOOR_ACT_S;
                else door_act_cnt <= door_act_cnt + 1'b1;
            end
        end else door_act_cnt <= 0;
    end

    assign door_auto_open_ok   = (door_m_open_cmd  && (door_act_cnt >= DOOR_ACT_S));
    assign door_auto_closed_ok = (door_m_close_cmd && (door_act_cnt >= DOOR_ACT_S));

    logic [3:0] door_open_cnt;
    logic       door_timer_active_r;
    assign door_timer_active = door_timer_active_r;

    always_ff @(posedge clk) begin
        if (rst) door_open_cnt <= 0;
        else if (door_timer_active) begin
            if (tick_1hz) begin
                if (door_open_cnt == OPEN_TIME_S) door_open_cnt <= OPEN_TIME_S;
                else door_open_cnt <= door_open_cnt + 1'b1;
            end
        end else door_open_cnt <= 0;
    end

    assign door_timer_done_auto = (door_timer_active && (door_open_cnt >= OPEN_TIME_S));

    // -------------------- Solicitudes -------------------------
    wire request_any_p = call_up_p | call_down_p;
    always_ff @(posedge clk) begin
        if (rst) target_floor <= 3'd0;
        else if (request_any_p) begin
            if (dest_floor < N_FLOORS) target_floor <= dest_floor;
            else                       target_floor <= N_FLOORS-1;
        end
    end

    // -------------------- FSM principal -----------------------
    typedef enum logic [2:0] {
        IDLE     = 3'd0,
        MOV_UP   = 3'd1,
        MOV_DN   = 3'd2,
        OPENING  = 3'd3,
        OPEN     = 3'd4,
        CLOSING  = 3'd5
    } state_t;

    state_t s, s_next;

    // Señales combinacionales
    logic       door_open_req, door_close_req, door_timer_req;
    logic [1:0] dir_code;
    logic       door_is_open, door_is_closed;
    logic [3:0] stc;

    // Dirección para display
    always_comb begin
        case (s)
            MOV_UP:   dir_code = 2'd1;
            MOV_DN:   dir_code = 2'd2;
            default:  dir_code = 2'd0;
        endcase
    end

    assign door_is_open   = sensor_door_open_ok_w;
    assign door_is_closed = sensor_door_closed_ok_w;

    // Control puerta/timer por estado
    always_comb begin
        door_open_req  = 1'b0;
        door_close_req = 1'b0;
        door_timer_req = 1'b0;
        unique case (s)
            IDLE:    ;
            MOV_UP:  ;
            MOV_DN:  ;
            OPENING: door_open_req  = 1'b1;
            OPEN:    door_timer_req = 1'b1;
            CLOSING: door_close_req = 1'b1;
        endcase
    end

    // Próximo estado
    always_comb begin
        s_next = s;
        if (alarm_on) begin
            s_next = door_is_open ? OPEN : OPENING;
        end else begin
            unique case (s)
                IDLE: begin
                    if (door_open_p)                    s_next = OPENING;
                    else if (door_close_p)              s_next = CLOSING;
                    else if (cur_floor < target_floor)  s_next = MOV_UP;
                    else if (cur_floor > target_floor)  s_next = MOV_DN;
                    else if (request_any_p)             s_next = OPENING;
                    else                                 s_next = IDLE;
                end
                MOV_UP:   s_next = (cur_floor >= target_floor) ? OPENING : MOV_UP;
                MOV_DN:   s_next = (cur_floor <= target_floor) ? OPENING : MOV_DN;
                OPENING:  s_next = door_is_open ? OPEN : OPENING;
                OPEN:     s_next = sensor_obstacle_w ? OPEN :
                                     (door_timer_done_w ? CLOSING : OPEN);
                CLOSING:  s_next = sensor_obstacle_w ? OPENING :
                                     (door_is_closed ? IDLE : CLOSING);
                default:  s_next = IDLE;
            endcase
        end
    end

    // Estado
    always_ff @(posedge clk) begin
        if (rst) s <= IDLE;
        else     s <= s_next;
    end

    // Motores traslación
    always_ff @(posedge clk) begin
        if (rst || alarm_on) begin
            motor_up   <= 1'b0;
            motor_down <= 1'b0;
        end else begin
            case (s_next)
                MOV_UP:   begin motor_up <= 1'b1; motor_down <= 1'b0; end
                MOV_DN:   begin motor_up <= 1'b0; motor_down <= 1'b1; end
                default:  begin motor_up <= 1'b0; motor_down <= 1'b0; end
            endcase
        end
    end

    // Timer puertas activo cuando estamos en OPEN
    always_ff @(posedge clk) begin
        if (rst) door_timer_active_r <= 1'b0;
        else     door_timer_active_r <= (s_next == OPEN);
    end

    // Comandos motor puerta
    assign door_m_open_cmd  = (s == OPENING);
    assign door_m_close_cmd = (s == CLOSING);

    // -------------------- Hora (vista alternativa) ------------
    wire [4:0] hh;
    wire [5:0] mm;
    timekeeper #(.INIT_HH(8'd12), .INIT_MM(8'd00))
    utime(.clk(clk), .rst(rst), .tick_1hz(tick_1hz), .hh(hh), .mm(mm));

    // -------------------- 7-seg -------------------------------
    wire mode = sw[0];

    wire [3:0] Hh = hh/10;
    wire [3:0] hH = hh%10;
    wire [3:0] Mm = mm/10;
    wire [3:0] mM = mm%10;

    logic [3:0] d3, d2, d1, d0;
    logic [2:0] a3, a2, a1, a0;
    logic [3:0] dp_mask;

    always_comb begin
        unique case (s)
            IDLE:    stc = 4'd0;
            MOV_UP:  stc = 4'd1;
            MOV_DN:  stc = 4'd2;
            OPENING: stc = 4'd3;
            OPEN:    stc = 4'd4;
            CLOSING: stc = 4'd5;
            default: stc = 4'd0;
        endcase
    end

    always_comb begin
        a3=0; a2=0; a1=0; a0=0; dp_mask=4'b0000;
        if (!mode) begin
            d3 = Hh; d2 = hH; d1 = Mm; d0 = mM;
            dp_mask = 4'b0010; // DP entre HH y MM
        end else begin
            d3 = {1'b0, cur_floor};                         // piso
            d2 = 4'h0; a2 = (dir_code==2'd1)?3'd1:(dir_code==2'd2)?3'd2:3'd5; // U/d/-
            d1 = 4'h0; a1 = door_is_open ? 3'd3 : 3'd4;     // o/C
            d0 = stc;                                       // estado
        end
    end

    seg7_mux umux(
        .clk(clk), .rst(rst), .tick_1khz(tick_1khz),
        .d3(d3), .d2(d2), .d1(d1), .d0(d0),
        .a3(a3), .a2(a2), .a1(a1), .a0(a0),
        .dp_mask(dp_mask), .an(an), .seg(seg)
    );

    // -------------------- LEDs diagnóstico --------------------
    logic req_latch;
    always_ff @(posedge clk) begin
        if (rst) req_latch <= 1'b0;
        else if (request_any_p) req_latch <= 1'b1;
        else if (tick_1hz) req_latch <= 1'b0;
    end

    assign led[2:0]  = cur_floor[2:0];
    assign led[3]    = req_latch;
    assign led[4]    = motor_up;
    assign led[5]    = motor_down;
    assign led[6]    = door_m_open_cmd;
    assign led[7]    = door_m_close_cmd;
    assign led[8]    = door_timer_active;
    assign led[9]    = sensor_obstacle_w;
    assign led[10]   = sensor_door_open_ok_w;
    assign led[11]   = sensor_door_closed_ok_w;
    assign led[12]   = 1'b0;
    assign led[13]   = 1'b0;
    assign led[14]   = alarm_on;
    assign led[15]   = mode;

endmodule

// =============================================================
// clk_enable: 1 Hz y ~1 kHz desde 100 MHz
// =============================================================
module clk_enable(
    input  wire clk,
    input  wire rst,
    output reg  tick_1hz,
    output reg  tick_1khz
);
    localparam integer DIV_1HZ  = 100_000_000;
    localparam integer DIV_1KHZ = 100_000;

    reg [26:0] c1;
    reg [16:0] c2;

    always @(posedge clk) begin
        if (rst) begin c1 <= 0; tick_1hz <= 1'b0; end
        else begin
            if (c1 == DIV_1HZ-1) begin c1 <= 0; tick_1hz <= 1'b1; end
            else begin c1 <= c1 + 1; tick_1hz <= 1'b0; end
        end
    end

    always @(posedge clk) begin
        if (rst) begin c2 <= 0; tick_1khz <= 1'b0; end
        else begin
            if (c2 == DIV_1KHZ-1) begin c2 <= 0; tick_1khz <= 1'b1; end
            else begin c2 <= c2 + 1; tick_1khz <= 1'b0; end
        end
    end
endmodule

// =============================================================
// seg7_decoder: 0-F + (U,d,o,C,-) activos-bajo
// =============================================================
module seg7_decoder(
    input  wire [3:0] nibble,
    input  wire [2:0] alpha, // 0=none,1=U,2=d,3=o,4=C,5='-'
    output reg  [6:0] seg
);
    always @* begin
        if (alpha!=3'd0) begin
            case(alpha)
                3'd1: seg = 7'b100_0001; // U
                3'd2: seg = 7'b010_0001; // d
                3'd3: seg = 7'b010_0010; // o
                3'd4: seg = 7'b100_0110; // C
                3'd5: seg = 7'b111_1110; // -
                default: seg = 7'b111_1111;
            endcase
        end else begin
            case(nibble)
                4'h0: seg = 7'b100_0000;
                4'h1: seg = 7'b111_1001;
                4'h2: seg = 7'b010_0100;
                4'h3: seg = 7'b011_0000;
                4'h4: seg = 7'b001_1001;
                4'h5: seg = 7'b001_0010;
                4'h6: seg = 7'b000_0010;
                4'h7: seg = 7'b111_1000;
                4'h8: seg = 7'b000_0000;
                4'h9: seg = 7'b001_0000;
                4'hA: seg = 7'b000_1000; // A
                4'hB: seg = 7'b000_0011; // b
                4'hC: seg = 7'b100_0110; // C
                4'hD: seg = 7'b010_0001; // d
                4'hE: seg = 7'b000_0110; // E
                4'hF: seg = 7'b000_1110; // F
            endcase
        end
    end
endmodule

// =============================================================
// seg7_mux: 4 dígitos + DP activos-bajo
// =============================================================
module seg7_mux(
    input  wire        clk,
    input  wire        rst,
    input  wire        tick_1khz,
    input  wire [3:0]  d3, d2, d1, d0,
    input  wire [2:0]  a3, a2, a1, a0,
    input  wire [3:0]  dp_mask,
    output reg  [3:0]  an,
    output reg  [7:0]  seg
);
    reg [1:0] sel;
    reg [3:0] nibble;
    reg [2:0] alpha;
    wire [6:0] seg_abc;

    seg7_decoder udec(.nibble(nibble), .alpha(alpha), .seg(seg_abc));

    always @(posedge clk) begin
        if (rst) sel <= 2'd0;
        else if (tick_1khz) sel <= sel + 2'd1;
    end

    always @* begin
        an = 4'b1111;
        case(sel)
            2'd0: begin an=4'b1110; nibble=d0; alpha=a0; end
            2'd1: begin an=4'b1101; nibble=d1; alpha=a1; end
            2'd2: begin an=4'b1011; nibble=d2; alpha=a2; end
            2'd3: begin an=4'b0111; nibble=d3; alpha=a3; end
        endcase
        seg[6:0] = seg_abc;
        case(sel)
            2'd0: seg[7] = ~dp_mask[0];
            2'd1: seg[7] = ~dp_mask[1];
            2'd2: seg[7] = ~dp_mask[2];
            2'd3: seg[7] = ~dp_mask[3];
        endcase
    end
endmodule

// =============================================================
// timekeeper: HH:MM
// =============================================================
module timekeeper #(
    parameter integer INIT_HH = 0,
    parameter integer INIT_MM = 0
)(
    input  wire clk,
    input  wire rst,
    input  wire tick_1hz,
    output reg  [4:0] hh,
    output reg  [5:0] mm
);
    reg [5:0] ss;
    always @(posedge clk) begin
        if (rst) begin
            hh <= INIT_HH[4:0];
            mm <= INIT_MM[5:0];
            ss <= 0;
        end else if (tick_1hz) begin
            if (ss == 59) begin
                ss <= 0;
                if (mm == 59) begin
                    mm <= 0;
                    if (hh == 23) hh <= 0; else hh <= hh + 1;
                end else mm <= mm + 1;
            end else ss <= ss + 1;
        end
    end
endmodule

// =============================================================
// debouncer
// =============================================================
module debouncer(
    input  wire clk,
    input  wire rst,
    input  wire noisy,
    output reg  clean
);
    reg [15:0] cnt;
    reg sync0, sync1;

    always @(posedge clk) begin
        sync0 <= noisy; sync1 <= sync0;
    end

    always @(posedge clk) begin
        if (rst) begin cnt<=0; clean<=0; end
        else if (clean == sync1) cnt <= 0;
        else begin
            cnt <= cnt + 1;
            if (&cnt) begin clean <= sync1; cnt <= 0; end
        end
    end
endmodule

// =============================================================
// one_shot
// =============================================================
module one_shot(
    input  wire clk,
    input  wire rst,
    input  wire in,
    output wire pulse
);
    reg d;
    always @(posedge clk) begin
        if (rst) d <= 1'b0; else d <= in;
    end
    assign pulse = in & ~d;
endmodule