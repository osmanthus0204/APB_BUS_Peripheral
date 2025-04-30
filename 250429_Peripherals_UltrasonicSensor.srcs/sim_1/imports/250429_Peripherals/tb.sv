`timescale 1ns / 1ps

module Ultrasonic_Periph_tb;
    // 시뮬레이션 파라미터
    parameter CLK_PERIOD = 10;  // 100MHz 클럭
    
    // 테스트 대상 모듈 신호
    logic        PCLK;
    logic        PRESET;
    logic [3:0]  PADDR;
    logic [31:0] PWDATA;
    logic        PWRITE;
    logic        PENABLE;
    logic        PSEL;
    logic [31:0] PRDATA;
    logic        PREADY;
    logic        echo;
    logic        trig;
    logic [2:0]  o_state;
    
    // 테스트 변수
    logic [11:0] distance;
    
    // DUT (Device Under Test) 인스턴스화
    Ultrasonic_Periph DUT (
        .PCLK(PCLK),
        .PRESET(PRESET),
        .PADDR(PADDR),
        .PWDATA(PWDATA),
        .PWRITE(PWRITE),
        .PENABLE(PENABLE),
        .PSEL(PSEL),
        .PRDATA(PRDATA),
        .PREADY(PREADY),
        .echo(echo),
        .trig(trig),
        .o_state(o_state)
    );
    
    // 클럭 생성
    always begin
        PCLK = 0;
        #(CLK_PERIOD/2);
        PCLK = 1;
        #(CLK_PERIOD/2);
    end
    
    // 테스트 시나리오를 위한 태스크
    
    // APB 쓰기 태스크
    task apb_write(input [3:0] addr, input [31:0] data);
        @(posedge PCLK);
        PSEL = 1;
        PWRITE = 1;
        PADDR = addr;
        PWDATA = data;
        @(posedge PCLK);
        PENABLE = 1;
        wait(PREADY);
        @(posedge PCLK);
        PSEL = 0;
        PENABLE = 0;
    endtask
    
    // APB 읽기 태스크
    task apb_read(input [3:0] addr, output [31:0] data);
        @(posedge PCLK);
        PSEL = 1;
        PWRITE = 0;
        PADDR = addr;
        @(posedge PCLK);
        PENABLE = 1;
        wait(PREADY);
        data = PRDATA;
        @(posedge PCLK);
        PSEL = 0;
        PENABLE = 0;
    endtask
    
    // echo 신호 생성 태스크 (거리에 따른 echo 신호 지속 시간 계산)
    // distance는 cm 단위
    task generate_echo(input int distance_cm);
        // 공식: e_count = (distance_cm * 58) / 10
        // 이는 (e_count * 10) / 58 = distance_cm의 역산
        int echo_duration;
        
        if (distance_cm < 2 || distance_cm > 400) begin
            echo_duration = (distance_cm < 2) ? 116 - 1 : 23200 + 1; // 에러 케이스
        end else begin
            echo_duration = (distance_cm * 58) / 10; // 정상 케이스, microseconds
        end
        
        // trig 신호 감지 후 echo 생성
        wait(trig == 1);
        wait(trig == 0);
        #10000; // 10us 대기 (실제 센서의 지연 시간)
        
        // echo 신호 생성
        echo = 1;
        #(echo_duration * 1000); // ns로 변환
        echo = 0;
    endtask
    
    // 테스트 시나리오
    initial begin
        // 초기화
        PRESET = 1;
        PSEL = 0;
        PENABLE = 0;
        PWRITE = 0;
        PADDR = 0;
        PWDATA = 0;
        echo = 0;
        distance = 0;
        
        // 리셋 해제
        #100
        PRESET = 0;
        
        // usr 레지스터(slv_reg0[0])를 1로 설정하여 센서 시작
        apb_write(4'h0, 32'h00000001);
        
        // 테스트 시나리오 1: 20cm 거리
        $display("테스트 시나리오 1: 20cm 거리 측정");
        generate_echo(20);
        
        // APB 읽기로 거리 확인
        #200000; // 충분한 시간 대기
        apb_read(4'h1, distance);
        $display("측정된 거리: %d.%d cm", distance/10, distance%10);
        
        // 테스트 시나리오 2: 100cm 거리
        $display("테스트 시나리오 2: 100cm 거리 측정");
        generate_echo(100);
        
        // APB 읽기로 거리 확인
        #400000; // 충분한 시간 대기
        apb_read(4'h1, distance);
        $display("측정된 거리: %d.%d cm", distance/10, distance%10);
        
        // 테스트 시나리오 3: 300cm 거리
        $display("테스트 시나리오 3: 300cm 거리 측정");
        generate_echo(300);
        
        // APB 읽기로 거리 확인
        #600000; // 충분한 시간 대기
        apb_read(4'h1, distance);
        $display("측정된 거리: %d.%d cm", distance/10, distance%10);
        
        // 테스트 시나리오 4: 에러 케이스 - 1cm (최소 유효 거리 미만)
        $display("테스트 시나리오 4: 에러 케이스 - 1cm (최소 유효 거리 미만)");
        generate_echo(1);
        
        // APB 읽기로 거리 확인
        #200000; // 충분한 시간 대기
        apb_read(4'h1, distance);
        $display("측정된 거리: %d.%d cm (에러 예상)", distance/10, distance%10);
        
        // 테스트 시나리오 5: 에러 케이스 - 450cm (최대 유효 거리 초과)
        $display("테스트 시나리오 5: 에러 케이스 - 450cm (최대 유효 거리 초과)");
        generate_echo(450);
        
        // APB 읽기로 거리 확인
        #800000; // 충분한 시간 대기
        apb_read(4'h1, distance);
        $display("측정된 거리: %d.%d cm (에러 예상)", distance/10, distance%10);
        
        // 시뮬레이션 종료
        #100000;
        $display("시뮬레이션 완료");
        $finish;
    end
    
    // 상태 모니터링
    always @(o_state) begin
        case(o_state)
            3'b000: $display("상태: IDLE");
            3'b001: $display("상태: TRIG");
            3'b010: $display("상태: RECEIVE");
            3'b011: $display("상태: COUNT");
            3'b100: $display("상태: RESULT");
            3'b101: $display("상태: IDLE_WAIT");
            3'b110: $display("상태: ERROR");
            default: $display("상태: UNKNOWN");
        endcase
    end
    
    // trig 신호 모니터링
    always @(trig) begin
        $display("trig 신호: %b, 시간: %t", trig, $time);
    end
    
endmodule
