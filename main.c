


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"

#include "io.c"

#include "I2C_core.h"
#include "terasic_includes.h"
#include "mipi_camera_config.h"
#include "mipi_bridge_config.h"
#include "auto_focus.h"


#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )



#define DEFAULT_LEVEL 2

#define MIPI_REG_PHYClkCtl		0x0056
#define MIPI_REG_PHYData0Ctl	0x0058
#define MIPI_REG_PHYData1Ctl	0x005A
#define MIPI_REG_PHYData2Ctl	0x005C
#define MIPI_REG_PHYData3Ctl	0x005E
#define MIPI_REG_PHYTimDly		0x0060
#define MIPI_REG_PHYSta			0x0062
#define MIPI_REG_CSIStatus		0x0064
#define MIPI_REG_CSIErrEn		0x0066
#define MIPI_REG_MDLSynErr		0x0068
#define MIPI_REG_FrmErrCnt		0x0080
#define MIPI_REG_MDLErrCnt		0x0090





/////////////////////////////////////////////////////// Reading Writting

void IOWR(void *base_address, int NumOfReg, uint32_t data){
	base_address += NumOfReg*4;
	*(uint32_t *)base_address = data;
}

uint32_t IORD(void *base_address, int NumOfReg){
	base_address += NumOfReg*4;
	return *(uint32_t *)base_address;
}
/* 
#define IORD(BASE, REGNUM) \
  __builtin_ldwio (__IO_CALC_ADDRESS_NATIVE ((BASE), (REGNUM)))
#define IOWR(BASE, REGNUM, DATA) \
  __builtin_stwio (__IO_CALC_ADDRESS_NATIVE ((BASE), (REGNUM)), (DATA))
 */


uint8_t IO_8_read(void *base_address, int NumOfReg){
	uint8_t data;
	base_address += NumOfReg*4;
	data = *(uint8_t *)base_address;
	return data;
}

uint16_t IO_16_read(void *base_address, int NumOfReg){
	uint16_t data;
	base_address += NumOfReg*4;
	data = *(uint16_t *)base_address;
	return data;
}

uint32_t IO_32_read(void *base_address, int NumOfReg){
	uint32_t data;
	base_address += NumOfReg*4;
	data = *(uint32_t *)base_address;
	return data;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






void mipi_clear_error(void){
	MipiBridgeRegWrite(MIPI_REG_CSIStatus,0x01FF); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLSynErr,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_FrmErrCnt,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLErrCnt, 0x0000); // clear error

  	MipiBridgeRegWrite(0x0082,0x00);
  	MipiBridgeRegWrite(0x0084,0x00);
  	MipiBridgeRegWrite(0x0086,0x00);
  	MipiBridgeRegWrite(0x0088,0x00);
  	MipiBridgeRegWrite(0x008A,0x00);
  	MipiBridgeRegWrite(0x008C,0x00);
  	MipiBridgeRegWrite(0x008E,0x00);
  	MipiBridgeRegWrite(0x0090,0x00);
}

void mipi_show_error_info(void){

	alt_u16 PHY_status, SCI_status, MDLSynErr, FrmErrCnt, MDLErrCnt;

	PHY_status = MipiBridgeRegRead(MIPI_REG_PHYSta);
	SCI_status = MipiBridgeRegRead(MIPI_REG_CSIStatus);
	MDLSynErr = MipiBridgeRegRead(MIPI_REG_MDLSynErr);
	FrmErrCnt = MipiBridgeRegRead(MIPI_REG_FrmErrCnt);
	MDLErrCnt = MipiBridgeRegRead(MIPI_REG_MDLErrCnt);
	printf("PHY_status=%xh, CSI_status=%xh, MDLSynErr=%xh, FrmErrCnt=%xh, MDLErrCnt=%xh\r\n", PHY_status, SCI_status, MDLSynErr,FrmErrCnt, MDLErrCnt);
}

void mipi_show_error_info_more(void){
    printf("FrmErrCnt = %d\n",MipiBridgeRegRead(0x0080));
    printf("CRCErrCnt = %d\n",MipiBridgeRegRead(0x0082));
    printf("CorErrCnt = %d\n",MipiBridgeRegRead(0x0084));
    printf("HdrErrCnt = %d\n",MipiBridgeRegRead(0x0086));
    printf("EIDErrCnt = %d\n",MipiBridgeRegRead(0x0088));
    printf("CtlErrCnt = %d\n",MipiBridgeRegRead(0x008A));
    printf("SoTErrCnt = %d\n",MipiBridgeRegRead(0x008C));
    printf("SynErrCnt = %d\n",MipiBridgeRegRead(0x008E));
    printf("MDLErrCnt = %d\n",MipiBridgeRegRead(0x0090));
    printf("FIFOSTATUS = %d\n",MipiBridgeRegRead(0x00F8));
    printf("DataType = 0x%04x\n",MipiBridgeRegRead(0x006A));
    printf("CSIPktLen = %d\n",MipiBridgeRegRead(0x006E));
}



bool MIPI_Init(void){
	bool bSuccess;


	bSuccess = oc_i2c_init_ex(h2p_lw_mipi_contrlr, 50*1000*1000,400*1000); //I2C: 400K
	if (!bSuccess)
		printf("failed to init MIPI- Bridge i2c\r\n");

    usleep(50*1000);
    MipiBridgeInit();

    usleep(500*1000);

//	bSuccess = oc_i2c_init_ex(h2p_lw_mipi_camera, 50*1000*1000,400*1000); //I2C: 400K
//	if (!bSuccess)
//		printf("failed to init MIPI- Camera i2c\r\n");

    MipiCameraInit();
//    MIPI_BIN_LEVEL(DEFAULT_LEVEL);

	
//    OV8865_FOCUS_Move_to(340);

//    oc_i2c_uninit(h2p_lw_mipi_camera);  // Release I2C bus , due to two I2C master shared!


 	usleep(1000);


    oc_i2c_uninit(h2p_lw_mipi_contrlr);

	return bSuccess;
}





uint32_t calc_address(uint32_t base_address){
	return ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + base_address ) & ( unsigned long)( HW_REGS_MASK ) );
	//h2p_lw_mix_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ALT_VIP_MIX_0_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	// 			virtual base: 0x72e23000,  		 alt_lwfpga_ofst: 0xff200000,  led_pio_base: 0x10040,  hardware_regs_mask: : 0x3ffffff
	//																 	FF210040 & 3ffffff
	//																		  3210040
}

int main() {

	void *virtual_base;
	int fd;
	int i;
	int loop_count;
	int led_direction;
	int led_mask;
	uint8_t start_mix = 0x01;
	uint8_t stop_mix = 0x00;
	int reg1, reg_value1, step;
	int *reg, *reg_value;
	void *h2p_lw_led_addr;
	void *h2p_lw_mipi_pwdn_n;
	void *h2p_lw_mipi_rest_n;
	void *h2p_lw_mix_addr;
	void *h2p_lw_frame_reader;
	
	
	reg1 = 0;
	uint16_t mix_data;
	uint32_t led_data = 0x0;
	
	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
	
	h2p_lw_led_addr		= virtual_base + calc_address(LED_BASE);		
	h2p_lw_mipi_pwdn_n	= virtual_base + calc_address(MIPI_PWDN_N_BASE);
	h2p_lw_mipi_rest_n	= virtual_base + calc_address(MIPI_RESET_N_BASE);
	h2p_lw_mix_addr		= virtual_base + calc_address(ALT_VIP_MIX_0_BASE);
	//h2p_lw_auto_focus	= virtual_base + calc_address(TERASIC_AUTO_FOCUS_0_BASE);
	h2p_lw_mipi_camera	= virtual_base + calc_address(I2C_OPENCORES_CAMERA_BASE);
	h2p_lw_mipi_contrlr	= virtual_base + calc_address(I2C_OPENCORES_MIPI_BASE);
	h2p_lw_frame_reader = virtual_base + calc_address(ALT_VIP_VFR_0_BASE);
	//h2p_lw_pio_reset_n	= virtual_base + calc_address(PIO_RESET_BASE);

	
	for(i=0; i<20; i++){
		mix_data = IO_16_read(h2p_lw_frame_reader,i);
		printf("Frame Reader data register %d: 0x%x \n",i,mix_data);
	}
	
	printf("\nInitializing Mixer...\n");
	
	IOWR(h2p_lw_mix_addr, 0 , stop_mix);
	usleep(100);
	IOWR(h2p_lw_mix_addr, 0 , start_mix);
	usleep(5000000); 
	

	 for(i=0; i<20; i++){
		mix_data = IO_16_read(h2p_lw_mix_addr,i);
		printf("Mix data register %d: 0x%x \n",i,mix_data);
	}
	 
	
	

	/* 
	///////////////////hardware reset///////////
	IOWR(h2p_lw_pio_reset_n,0,0x00);
	usleep(2000); 
	IOWR(h2p_lw_pio_reset_n,0,0x01); */
	
	
	IOWR(h2p_lw_led_addr,0,0x3ff);
	printf("\nLED_data: 0x%x\n",led_data);
	
	
	printf("DE1-SoC D8M VGA Demo\n");
	IOWR(h2p_lw_mipi_pwdn_n, 0x00, 0x00);
	IOWR(h2p_lw_mipi_rest_n, 0x00, 0x00);

	usleep(2000);
	IOWR(h2p_lw_mipi_pwdn_n, 0x00, 0xFF);
	usleep(2000);
	IOWR(h2p_lw_mipi_rest_n, 0x00, 0xFF);


	usleep(2000);

	printf("\nStart of...... MIPI_Init Initialization!\r\n");
	// MIPI Init
	if (!MIPI_Init()){
		printf("MIPI_Init Init failed!\r\n");
	}else{
		printf("MIPI_Init Init successfully!\r\n");
	}

	
	
//   while(1){
 	mipi_clear_error();
	usleep(50*1000);
 	mipi_clear_error();
	usleep(1000*1000);
	mipi_show_error_info();
	mipi_show_error_info_more();
	printf("\n");
//   }

	usleep(1000*1000*10);
	//Focus_Init();
	
	
/* 	
#if 0  // focus sweep
	    printf("\nFocus sweep\n");
 	 	alt_u16 ii= 350;
 	    alt_u8  dir = 0;
 	 	while(1){
 	 		if(ii< 50) dir = 1;
 	 		else if (ii> 1000) dir =0;

 	 		if(dir) ii += 20;
 	 		else    ii -= 20;

 	    	printf("%d\n",ii);
 	     OV8865_FOCUS_Move_to(ii);
 	     usleep(50*1000);
 	    }
#endif */






    //////////////////////////////////////////////////////////
/*         alt_u16 bin_level = DEFAULT_LEVEL;
        alt_u8  manual_focus_step = 10;
        alt_u16  current_focus = 300; */
        
		
		
	
	
	
	
	
	loop_count = 0;
	led_mask = 0x01;
	led_direction = 0; // 0: left to right direction
	
	
	
	
		
	 

	for(i=0; i<20; i++){
		mix_data = IO_16_read(h2p_lw_mix_addr,i);
		printf("Mix data register %d: 0x%x \n",i,mix_data);
	}
	 
	
	while(reg1 != -1){
	
		printf("Tell me witch register you want to configure: ");
		scanf("%d",&reg);
		printf("\n");
		reg1 = reg;
		
		
		printf("Ready for configure register %d of vip mix. Give value: ",reg1);
		scanf("%d",&reg_value);
		reg_value1 = reg_value;
		
		if (reg1 == 4){
			IOWR(h2p_lw_mix_addr, 0 , stop_mix);
		
			if (reg_value1 == 0){ 
				IOWR(h2p_lw_mix_addr,reg1,stop_mix);  //disable mix input 1
			}
			else{
				IOWR(h2p_lw_mix_addr,reg1,start_mix); //enable mix input 1
			}
			
			IOWR(h2p_lw_mix_addr, 0 , start_mix);
		}
		else if (reg1 == 0){
			if (reg_value1 == 0){
				IOWR(h2p_lw_mix_addr,reg1,stop_mix);
			}
			else{
				IOWR(h2p_lw_mix_addr,reg1,start_mix);
			}
		}
		else{ 
			IOWR(h2p_lw_mix_addr,reg1,reg_value1);
		}
		
		for(i=0; i<20; i++){
			mix_data = IO_16_read(h2p_lw_mix_addr,i);
			printf("Mix data register %d: 0x%x \n",i,mix_data);
		}
			
	}
	

	 
	 while( loop_count < 10 ) {
		
		// control led
		*(uint32_t *)h2p_lw_led_addr = ~led_mask; 
		// wait 100ms
		usleep( 100*1000 );
		
		// update led mask
		if (led_direction == 0){
			led_mask <<= 1;
			if (led_mask == (0x01 << (LED_DATA_WIDTH-1)))
				 led_direction = 1;
		}else{
			led_mask >>= 1;
			if (led_mask == 0x01){ 
				led_direction = 0;
				loop_count++;
			}
		}
		
	} // while
	 

	
	//Releasing I2C
	if(oc_i2c_uninit(h2p_lw_mipi_camera) == 1){
		printf( "\nClearing I2C_OPENCORES_CAMERA_BASE...\n" );
	}
	 
	if(oc_i2c_uninit(h2p_lw_mipi_contrlr) == 1){
		printf( "\nClearing I2C_OPENCORES_MIPI_BASE...\n" );
	}
	
	
	usleep(2000);
	IOWR(h2p_lw_mipi_pwdn_n, 0x00, 0x00);
	usleep(2000);
	IOWR(h2p_lw_mipi_rest_n, 0x00, 0x00);

	
	
	// clean up our memory mapping and exit
	
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
