#include <stdint.h>
#include <stdbool.h>

#ifndef YOSYS_SIMPLEC_SIGNAL1_T
#define YOSYS_SIMPLEC_SIGNAL1_T
typedef struct {
  uint8_t value_0_0 : 1;
} signal1_t;
#endif

#ifndef YOSYS_SIMPLEC_COUNTER_STATE_T
#define YOSYS_SIMPLEC_COUNTER_STATE_T
struct counter_state_t
{
  // Input Ports
  signal1_t clk; // clk
  signal1_t en; // en
  signal1_t rst; // rst

  // Output Ports
  signal1_t count_0_; // count[0]
  signal1_t count_1_; // count[1]

  // Internal Wires
  signal1_t _abc_129_auto_rtlil_cc_2985_MuxGate_124; // $abc$129$auto$rtlil.cc:2985:MuxGate$124
  signal1_t _abc_129_auto_rtlil_cc_2985_MuxGate_128; // $abc$129$auto$rtlil.cc:2985:MuxGate$128
  signal1_t _abc_129_new_n10; // $abc$129$new_n10
  signal1_t _abc_129_new_n11; // $abc$129$new_n11
  signal1_t _abc_129_new_n12; // $abc$129$new_n12
  signal1_t _abc_129_new_n14; // $abc$129$new_n14
  signal1_t _abc_129_new_n15; // $abc$129$new_n15
  signal1_t _abc_129_new_n16; // $abc$129$new_n16
  signal1_t _abc_129_new_n7; // $abc$129$new_n7
  signal1_t _abc_129_new_n8; // $abc$129$new_n8
  signal1_t _abc_129_new_n9; // $abc$129$new_n9
};
#endif

static void counter_init(struct counter_state_t *state)
{
}

static void counter_eval(struct counter_state_t *state)
{
  // Updated signal in counter: \clk
  // Updated signal in counter: \rst
  // Updated signal in counter: \en
}
