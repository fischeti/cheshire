// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Thomas Benz <tbenz@iis.ee.ethz.ch>
// Andreas Kuster <kustera@ethz.ch>
// Paul Scheffler <paulsc@iis.ee.ethz.ch>
// Fabian Schuiki <fschuiki@iis.ee.ethz.ch>

/// iDMA desc64 frontend wrapper for integration into Cheshire.
module cheshire_idma_desc64_wrap #(
  parameter int unsigned AxiAddrWidth     = 0,
  parameter int unsigned AxiDataWidth     = 0,
  parameter int unsigned AxiIdWidth       = 0,
  parameter int unsigned AxiUserWidth     = 0,
  parameter int unsigned AxiSlvIdWidth    = 0,
  parameter int unsigned AxiMaxWriteTxns  = 4,
  parameter int unsigned AxiMaxReadTxns   = 4,
  parameter int unsigned NumAxInFlight    = 0,
  parameter int unsigned MemSysDepth      = 0,
  parameter int unsigned InputFifoDepth   = 8,
  parameter int unsigned PendingFifoDepth = 8,
  parameter int unsigned NSpeculation     = 4,
  parameter bit          RAWCouplingAvail = 0,
  parameter type         axi_mst_req_t   = logic,
  parameter type         axi_mst_rsp_t   = logic,
  parameter type         axi_slv_req_t   = logic,
  parameter type         axi_slv_rsp_t   = logic
) (
  input  logic          clk_i,
  input  logic          rst_ni,
  input  logic          testmode_i,
  // Data transfer AXI master (→ AxiIn.dma)
  output axi_mst_req_t  axi_mst_req_o,
  input  axi_mst_rsp_t  axi_mst_rsp_i,
  // Descriptor fetch AXI master (→ AxiIn.dma_desc)
  output axi_mst_req_t  axi_desc_req_o,
  input  axi_mst_rsp_t  axi_desc_rsp_i,
  // Config register AXI slave (← AxiOut.dma, via atomics)
  input  axi_slv_req_t  axi_slv_req_i,
  output axi_slv_rsp_t  axi_slv_rsp_o,
  // IRQ output (TODO: connect to interrupt fabric)
  output logic          irq_o
);

  `include "axi/typedef.svh"
  `include "apb/typedef.svh"
  `include "idma/typedef.svh"

  localparam int unsigned TfLenWidth  = 32;
  localparam int unsigned BufferDepth = 3;

  typedef logic [AxiAddrWidth-1:0]   addr_t;
  typedef logic [AxiDataWidth-1:0]   data_t;
  typedef logic [AxiDataWidth/8-1:0] strb_t;
  typedef logic [AxiIdWidth-1:0]     id_t;
  typedef logic [AxiUserWidth-1:0]   user_t;
  typedef logic [TfLenWidth-1:0]     tf_len_t;

  // AXI4-Lite types (intermediate for AXI→APB conversion)
  `AXI_LITE_TYPEDEF_ALL(dma_axi_lite, addr_t, data_t, strb_t)

  // APB types (64-bit data width, matching idma_desc64_reg.rdl regwidth)
  `APB_TYPEDEF_ALL(dma_apb, addr_t, data_t, strb_t)

  // AXI channel types for iDMA metadata
  `AXI_TYPEDEF_AW_CHAN_T(axi_aw_chan_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_AR_CHAN_T(axi_ar_chan_t, addr_t, id_t, user_t)
  `AXI_TYPEDEF_R_CHAN_T(axi_r_chan_t, data_t, id_t, user_t)

  // iDMA request/response types
  `IDMA_TYPEDEF_FULL_REQ_T(idma_req_t, id_t, addr_t, tf_len_t)
  `IDMA_TYPEDEF_FULL_RSP_T(idma_rsp_t, addr_t)

  typedef struct packed { axi_ar_chan_t ar_chan; } axi_read_meta_channel_t;
  typedef struct packed { axi_read_meta_channel_t axi; } read_meta_channel_t;
  typedef struct packed { axi_aw_chan_t aw_chan; } axi_write_meta_channel_t;
  typedef struct packed { axi_write_meta_channel_t axi; } write_meta_channel_t;

  // Address decoder rule type for axi_lite_to_apb (single slave, full address range)
  typedef struct packed {
    logic [0:0] idx;
    addr_t      start_addr;
    addr_t      end_addr;
  } apb_rule_t;

  // Internal signals
  dma_axi_lite_req_t    axi_lite_req;
  dma_axi_lite_resp_t   axi_lite_rsp;
  dma_apb_req_t  [0:0]  apb_req;
  dma_apb_resp_t [0:0]  apb_rsp;

  idma_req_t             idma_req;
  logic                  idma_req_valid;
  logic                  idma_req_ready;
  idma_rsp_t             idma_rsp;
  logic                  idma_rsp_valid;
  logic                  idma_rsp_ready;
  idma_pkg::idma_busy_t  idma_busy;

  axi_mst_req_t          axi_read_req, axi_write_req;
  axi_mst_rsp_t          axi_read_rsp, axi_write_rsp;

  // AXI full → AXI-Lite (handles ATOPs and bursts internally)
  axi_to_axi_lite #(
    .AxiAddrWidth    ( AxiAddrWidth          ),
    .AxiDataWidth    ( AxiDataWidth          ),
    .AxiIdWidth      ( AxiSlvIdWidth         ),
    .AxiUserWidth    ( AxiUserWidth          ),
    .AxiMaxWriteTxns ( AxiMaxWriteTxns       ),
    .AxiMaxReadTxns  ( AxiMaxReadTxns        ),
    .full_req_t      ( axi_slv_req_t         ),
    .full_resp_t     ( axi_slv_rsp_t         ),
    .lite_req_t      ( dma_axi_lite_req_t    ),
    .lite_resp_t     ( dma_axi_lite_resp_t   )
  ) i_axi_to_axi_lite (
    .clk_i,
    .rst_ni,
    .test_i       ( testmode_i    ),
    .slv_req_i    ( axi_slv_req_i ),
    .slv_resp_o   ( axi_slv_rsp_o ),
    .mst_req_o    ( axi_lite_req  ),
    .mst_resp_i   ( axi_lite_rsp  )
  );

  // AXI-Lite → APB (64-bit data, single slave, full address space rule)
  axi_lite_to_apb #(
    .NoApbSlaves      ( 1                    ),
    .NoRules          ( 1                    ),
    .AddrWidth        ( AxiAddrWidth         ),
    .DataWidth        ( AxiDataWidth         ),
    .axi_lite_req_t   ( dma_axi_lite_req_t   ),
    .axi_lite_resp_t  ( dma_axi_lite_resp_t  ),
    .apb_req_t        ( dma_apb_req_t        ),
    .apb_resp_t       ( dma_apb_resp_t       ),
    .rule_t           ( apb_rule_t           )
  ) i_axi_lite_to_apb (
    .clk_i,
    .rst_ni,
    .axi_lite_req_i   ( axi_lite_req ),
    .axi_lite_resp_o  ( axi_lite_rsp ),
    .apb_req_o        ( apb_req      ),
    .apb_resp_i       ( apb_rsp      ),
    // Full address range → slave 0; routing already done by crossbar
    .addr_map_i       ( '{'{idx: '0, start_addr: '0, end_addr: '0}} )
  );

  // iDMA desc64 frontend
  idma_desc64_top #(
    .AddrWidth        ( AxiAddrWidth              ),
    .DataWidth        ( AxiDataWidth              ),
    .AxiIdWidth       ( AxiIdWidth                ),
    .idma_req_t       ( idma_req_t                ),
    .idma_rsp_t       ( idma_rsp_t                ),
    .apb_req_t        ( dma_apb_req_t             ),
    .apb_rsp_t        ( dma_apb_resp_t            ),
    .axi_req_t        ( axi_mst_req_t             ),
    .axi_rsp_t        ( axi_mst_rsp_t             ),
    .axi_ar_chan_t    ( axi_ar_chan_t              ),
    .axi_r_chan_t     ( axi_r_chan_t               ),
    .InputFifoDepth   ( InputFifoDepth            ),
    .PendingFifoDepth ( PendingFifoDepth          ),
    .BackendDepth     ( NumAxInFlight + BufferDepth ),
    .NSpeculation     ( NSpeculation              )
  ) i_idma_desc64_top (
    .clk_i,
    .rst_ni,
    .master_req_o      ( axi_desc_req_o   ),
    .master_rsp_i      ( axi_desc_rsp_i   ),
    .axi_ar_id_i       ( '0               ),
    .axi_aw_id_i       ( '0               ),
    .slave_req_i       ( apb_req[0]       ),
    .slave_rsp_o       ( apb_rsp[0]       ),
    .idma_req_o        ( idma_req         ),
    .idma_req_valid_o  ( idma_req_valid   ),
    .idma_req_ready_i  ( idma_req_ready   ),
    .idma_rsp_i        ( idma_rsp         ),
    .idma_rsp_valid_i  ( idma_rsp_valid   ),
    .idma_rsp_ready_o  ( idma_rsp_ready   ),
    .idma_busy_i       ( |idma_busy       ),
    .irq_o             ( irq_o            )
  );

  // iDMA backend (AXI read+write)
  idma_backend_rw_axi #(
    .CombinedShifter      ( 1'b0                      ),
    .DataWidth            ( AxiDataWidth              ),
    .AddrWidth            ( AxiAddrWidth              ),
    .AxiIdWidth           ( AxiIdWidth                ),
    .UserWidth            ( AxiUserWidth              ),
    .TFLenWidth           ( TfLenWidth                ),
    .MaskInvalidData      ( 1                         ),
    .BufferDepth          ( BufferDepth               ),
    .RAWCouplingAvail     ( RAWCouplingAvail           ),
    .HardwareLegalizer    ( 1                         ),
    .RejectZeroTransfers  ( 1                         ),
    .ErrorCap             ( idma_pkg::NO_ERROR_HANDLING ),
    .PrintFifoInfo        ( 0                         ),
    .NumAxInFlight        ( NumAxInFlight             ),
    .MemSysDepth          ( MemSysDepth               ),
    .idma_req_t           ( idma_req_t                ),
    .idma_rsp_t           ( idma_rsp_t                ),
    .idma_eh_req_t        ( idma_pkg::idma_eh_req_t   ),
    .idma_busy_t          ( idma_pkg::idma_busy_t     ),
    .axi_req_t            ( axi_mst_req_t             ),
    .axi_rsp_t            ( axi_mst_rsp_t             ),
    .write_meta_channel_t ( write_meta_channel_t      ),
    .read_meta_channel_t  ( read_meta_channel_t       )
  ) i_idma_backend (
    .clk_i,
    .rst_ni,
    .testmode_i,
    .idma_req_i       ( idma_req       ),
    .req_valid_i      ( idma_req_valid ),
    .req_ready_o      ( idma_req_ready ),
    .idma_rsp_o       ( idma_rsp       ),
    .rsp_valid_o      ( idma_rsp_valid ),
    .rsp_ready_i      ( idma_rsp_ready ),
    .idma_eh_req_i    ( '0             ),
    .eh_req_valid_i   ( '0             ),
    .eh_req_ready_o   (                ),
    .axi_read_req_o   ( axi_read_req   ),
    .axi_read_rsp_i   ( axi_read_rsp   ),
    .axi_write_req_o  ( axi_write_req  ),
    .axi_write_rsp_i  ( axi_write_rsp  ),
    .busy_o           ( idma_busy      )
  );

  // Join AXI read and write channels into a single master port
  axi_rw_join #(
    .axi_req_t   ( axi_mst_req_t ),
    .axi_resp_t  ( axi_mst_rsp_t )
  ) i_axi_rw_join (
    .clk_i,
    .rst_ni,
    .slv_read_req_i    ( axi_read_req  ),
    .slv_read_resp_o   ( axi_read_rsp  ),
    .slv_write_req_i   ( axi_write_req ),
    .slv_write_resp_o  ( axi_write_rsp ),
    .mst_req_o         ( axi_mst_req_o ),
    .mst_resp_i        ( axi_mst_rsp_i )
  );

endmodule
