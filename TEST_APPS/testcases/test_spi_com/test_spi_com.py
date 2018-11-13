"""
Copyright 2018 ARM Limited
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
from icetea_lib.bench import Bench
from icetea_lib.bench import TestStepError
from icetea_lib.tools.tools import test_case
from icetea_lib.TestStepError import TestStepFail

SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE = 0
SPI_MODE_IDLE_LOW_SAMPLE_SECOND_EDGE = 1
SPI_MODE_IDLE_HIGH_SAMPLE_FIRST_EDGE = 2
SPI_MODE_IDLE_HIGH_SAMPLE_SECOND_EDGE = 3

SPI_BIT_ORDERING_MSB_FIRST = 0
SPI_BIT_ORDERING_LSB_FIRST = 1

SPI_BUFFERS_EQUAL = 0
SPI_BUFFERS_MASTER_TX_GT_RX = 1
SPI_BUFFERS_MASTER_TX_LT_RX = 2
SPI_BUFFERS_SLAVE_TX_GT_RX = 3
SPI_BUFFERS_SLAVE_TX_LT_RX = 4
SPI_BUFFERS_ONE_SYM = 5
SPI_BUFFERS_LONG = 6

SPI_SYNC_MASTER_SYNC_SLAVE = 3
SPI_ASYNC_MASTER_SYNC_SLAVE = 2
SPI_SYNC_MASTER_ASYNC_SLAVE = 1
SPI_ASYNC_MASTER_ASYNC_SLAVE = 0

FULL_DUPLEX = 0
HALF_DUPLEX = 1

FREQ_MIN = 0
FREQ_MAX = -1

CONFIG_STRING  = "symbol_size %d mode %d bit_ordering %d freq_hz %d buffers %d master_tx_defined %s master_rx_defined %s slave_tx_defined %s slave_rx_defined %s auto_ss %s duplex %d sync %d"

class SPIComBaseTestEnv(Bench):
    def __init__(self, **kwargs):
        testcase_args = {
                       'name': "test_spi_com",
                       'title': "SPI Communication Test",
                       'status': "released",
                       'purpose': "Verify new SPI HAL API",
                       'component': ["SPI"],
                       'type': "compatibility",
                       'requirements': {
                           "duts": {
                               '*': {
                                   "count": 2,
                                   "type": "hardware"
                               },
                               "1": {"nick": "master",
                                     "application": {
                                       "name": "TEST_APPS-device-spi_master"
                                   }},
                               "2": {"nick": "slave",
                                     "application": {
                                       "name": "TEST_APPS-device-spi_slave"
                                   }}
                           }
                       } }

        testcase_args.update(kwargs)
        Bench.__init__(self, **testcase_args)

    def perform_test(self, tc_config):
        
        config_str = CONFIG_STRING % (tc_config['symbol_size'], 
                                      tc_config['mode'], 
                                      tc_config['bit_ordering'],
                                      tc_config['freq_hz'], 
                                      tc_config['buffers'], 
                                      tc_config['master_tx_defined'], 
                                      tc_config['master_rx_defined'], 
                                      tc_config['slave_tx_defined'], 
                                      tc_config['slave_rx_defined'], 
                                      tc_config['auto_ss'], 
                                      tc_config['duplex'], 
                                      tc_config['sync'])
        
        resp_master = self.command("master", "validate_config %s" % config_str)
        resp_slave = self.command("slave", "validate_config %s" % config_str)
        
        if (not resp_master.verify_trace("SKIP", break_in_fail=False) and not resp_master.verify_trace("SKIP", break_in_fail=False)):
            resp_slave = self.command("slave", "init_test")
            resp_master = self.command("master", "init_test")

            
            async_cmd_slave = self.command("slave", "exec_test", asynchronous=True)
            resp_master = self.command("master", "exec_test", report_cmd_fail=False)
            
            resp_slave = self.wait_for_async_response("exec_test", async_cmd_slave)
            
            master_error = resp_master.verify_trace("ERROR", break_in_fail=False)
            slave_error = resp_slave.verify_trace("ERROR", break_in_fail=False)
            
            master_timeout = resp_master.verify_trace("timeout", break_in_fail=False)
            slave_timeout = resp_slave.verify_trace("timeout", break_in_fail=False)
            
            resp_master = self.command("master", "finish_test")
            resp_slave = self.command("slave", "finish_test")
            
            if(master_error or slave_error):
                raise TestStepFail("Communication failure")
                
            if(master_timeout or slave_timeout):
                raise TestStepTimeout("Timeout");
        else:
            self.logger.info("TEST CASE SKIPPED")


    def setup(self):
        # setup code
        pass

    def teardown(self):
        # teardown code
        pass

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_DEFAULT_CFG",
           title="default config")
def SPI_COM_DEFAULT_CFG(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_SYM_SIZE_1",
           title="symbol size: 1 bit")
def SPI_COM_SYM_SIZE_1(self):
    self.perform_test({'symbol_size': 1 , 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_SYM_SIZE_7",
           title="symbol size: 7 bit")
def SPI_COM_SYM_SIZE_7(self):
    self.perform_test({'symbol_size': 7 , 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_SYM_SIZE_9",
           title="symbol size: 9 bit")
def SPI_COM_SYM_SIZE_9(self):
    self.perform_test({'symbol_size': 9 , 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_SYM_SIZE_15",
           title="symbol size: 15 bit")
def SPI_COM_SYM_SIZE_15(self):
    self.perform_test({'symbol_size': 15, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_SYM_SIZE_16",
           title="symbol size: 16 bit")
def SPI_COM_SYM_SIZE_16(self):
    self.perform_test({'symbol_size': 16, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })
    
@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_SYM_SIZE_17",
           title="symbol size: 17 bit")
def SPI_COM_SYM_SIZE_17(self):
    self.perform_test({'symbol_size': 17, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })   

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_SYM_SIZE_31",
           title="symbol size: 31 bit")
def SPI_COM_SYM_SIZE_31(self):
    self.perform_test({'symbol_size': 31, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })  

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_SYM_SIZE_32",
           title="symbol size: 32 bit")
def SPI_COM_SYM_SIZE_32(self):
    self.perform_test({'symbol_size': 32, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })  

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_MODE_CLK_IDLE_LOW_SEC_EDGE",
           title="mode: clk idle low/sample on second clk egde")
def SPI_COM_MODE_CLK_IDLE_LOW_SEC_EDGE(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_SECOND_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_MODE_CLK_IDLE_HIGH_FIRST_EDGE",
           title="mode: clk idle high/sample on first clk egde")
def SPI_COM_MODE_CLK_IDLE_HIGH_FIRST_EDGE(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_HIGH_SAMPLE_FIRST_EDGE , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_MODE_CLK_IDLE_HIGH_SEC_EDGE",
           title="mode: clk idle high/sample on second clk egde")
def SPI_COM_MODE_CLK_IDLE_HIGH_SEC_EDGE(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_HIGH_SAMPLE_SECOND_EDGE, 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BIT_ORDERING_LSB_FIRST",
           title="bit ordering: LBS first")
def SPI_COM_BIT_ORDERING_LSB_FIRST(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_LSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_FREQ_MIN_REQUIRED",
           title="freq: min required")
def SPI_COM_FREQ_MIN_REQUIRED(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 200000    , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_FREQ_MAX_REQUIRED",
           title="freq: max required")
def SPI_COM_FREQ_MAX_REQUIRED(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 2000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })
    
@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_FREQ_MIN_CAPABILITIES",
           title="freq: min defined by capabilities")
def SPI_COM_FREQ_MIN_CAPABILITIES(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': FREQ_MIN  , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })
    
@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_FREQ_MAX_CAPABILITIES",
           title="freq: max defined by capabilities")
def SPI_COM_FREQ_MIN_CAPABILITIES(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': FREQ_MAX  , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_MASTER_TX_GT_RX",
           title="buffers: master TX > master RX")
def SPI_COM_BUFFERS_MASTER_TX_GT_RX(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_MASTER_TX_GT_RX , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })
    
@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_MASTER_TX_LT_RX",
           title="buffers: master TX < master RX")
def SPI_COM_BUFFERS_MASTER_TX_LT_RX(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_MASTER_TX_LT_RX , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })
    
@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_SLAVE_TX_GT_RX",
           title="buffers: slave TX > slave RX")
def SPI_COM_BUFFERS_SLAVE_TX_GT_RX(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_SLAVE_TX_GT_RX  , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_SLAVE_TX_LT_RX",
           title="buffers: slave TX < slave RX")
def SPI_COM_BUFFERS_SLAVE_LT_GT_RX(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_SLAVE_TX_LT_RX  , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })
    
@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_MASTER_TX_UNDEF",
           title="buffers: master TX undefined")
def SPI_COM_BUFFERS_MASTER_TX_UNDEF(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'false', 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })
    
@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_MASTER_RX_UNDEF",
           title="buffers: master RX undefined")
def SPI_COM_BUFFERS_MASTER_RX_UNDEF(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'false', 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })
    
@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_SLAVE_TX_UNDEF",
           title="buffers: slave TX undefined")
def SPI_COM_BUFFERS_SLAVE_TX_UNDEF(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'false', 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_SLAVE_RX_UNDEF",
           title="buffers: slave RX undefined")
def SPI_COM_BUFFERS_SLAVE_RX_UNDEF(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'false', 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_LONG",
           title="buffers: transfer large number of symbols")
def SPI_COM_BUFFERS_LONG(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_LONG            , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_BUFFERS_ONE_SYM",
           title="buffers: transfer only one symbol")
def SPI_COM_BUFFERS_ONE_SYM(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_ONE_SYM         , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_AUTO_SS",
           title="slave select: auto ss handling by master")
def SPI_COM_AUTO_SS(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'true' , 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_HALF_COM_DUPLEX",
           title="half duplex transfer")
def SPI_HALF_COM_DUPLEX(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': HALF_DUPLEX,'sync': SPI_SYNC_MASTER_SYNC_SLAVE  })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_ASYNC_MASTER",
           title="master async mode")
def SPI_COM_ASYNC_MASTER(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_ASYNC_MASTER_SYNC_SLAVE })

@test_case(SPIComBaseTestEnv, 
           name="SPI_COM_ASYNC_SLAVE",
           title="slave async mode")
def SPI_COM_ASYNC_SLAVE(self):
    self.perform_test({'symbol_size': 8, 'mode': SPI_MODE_IDLE_LOW_SAMPLE_FIRST_EDGE  , 'bit_ordering': SPI_BIT_ORDERING_MSB_FIRST, 'freq_hz': 1000000   , 'buffers': SPI_BUFFERS_EQUAL           , 'master_tx_defined': 'true' , 'master_rx_defined': 'true' , 'slave_tx_defined': 'true' , 'slave_rx_defined': 'true' , 'auto_ss': 'false', 'duplex': FULL_DUPLEX     ,'sync': SPI_SYNC_MASTER_ASYNC_SLAVE })
