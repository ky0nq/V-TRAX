`timescale 1ns / 1ps

module gpio (
    input  wire [15:0] cr,       // Control Register: 1=출력모드, 0=입력모드
    output wire [15:0] idr,      // Input Data Register: 핀의 현재 읽히는 값
    input  wire [15:0] aodr,     // Alternate Output Data Register: 출력모드일 때 내보낼 값
    // ===== external port =====
    inout  wire [15:0] io_port
);

    genvar i;

    generate
        for (i = 0; i < 16; i = i + 1) begin : GPIO_PIN
            assign io_port[i] = cr[i] ? aodr[i] : 1'bz;
            assign idr[i]     = io_port[i];
        end
    endgenerate

endmodule