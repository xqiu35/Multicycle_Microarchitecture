/***************************************************************/
/*                                                             */
/* LC-3b Simulator (Adapted from Prof. Yale Patt at UT Austin) */
/*                                                             */
/***************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {                                                  
    IRD,
    COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); } 
int GetLSHF1(int *x)         { return(x[LSHF1]); }

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x08000 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */ 
} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {                                                    
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {                                                

  eval_micro_sequencer();   
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {                                      
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {                                                     
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/ 
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {          
    int address; /* this is a byte address */

    printf("\nMemory content [0x%04x..0x%04x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%04x (%d) : 0x%02x%02x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%04x..0x%04x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%04x (%d) : 0x%02x%02x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */   
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {                               
    int k; 

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%04x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%04x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%04x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%04x\n", BUS);
    printf("MDR          : 0x%04x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%04x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%04x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%04x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%04x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%04x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%04x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%04x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%04x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%04x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */  
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {                         
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */ 
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {                 
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/************************************************************/
/*                                                          */
/* Procedure : init_memory                                  */
/*                                                          */
/* Purpose   : Zero out the memory array                    */
/*                                                          */
/************************************************************/
void init_memory() {                                           
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {                   
    FILE * prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *ucode_filename, char *program_filename, int num_prog_files) { 
    int i;
    init_control_store(ucode_filename);

    init_memory();
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(program_filename);
	while(*program_filename++ != '\0');
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {                              
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 3) {
	printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv[1], argv[2], argc - 2);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* --------- DO NOT MODIFY THE CODE ABOVE THIS LINE -----------*/
/***************************************************************/

/***************************************************************/
/* You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/

//         Xiaofei's Implementation starts from here

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
///////////////////////// Variables //////////////////////////////

int *x = & CURRENT_LATCHES.MICROINSTRUCTION[0];

int WE1,WE0;

int COUNT=0;

int vALU, vMAR, vMDR, vSHF, vPC;


//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////// Addition Functions //////////////////////////////

int Get_Bits(int value, int start, int end){
  int result;
  result = value >> end;
  result = result % ( 1 << ( start - end + 1 ) );
  return result; 
}

int SEXT(int value, int topbit){
  int shift = sizeof(int)*8 - topbit; 
  return (value << shift )>> shift;
}

int ZEXT(int value){
return value & 0xFFFF;
}

int LSHF(int value, int amount){
  return (value << amount) & 0xFFFF;
}

int RSHF(int value, int amount, int topbit ){
  int mask;
  mask = 1 << amount;
  mask -= 1;
  mask = mask << ( 16 -amount );
  
  return ((value >> amount) & ~mask) | ((topbit)?(mask):0); /* TBD */
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

void eval_micro_sequencer() {


   int OPCODE = Get_Bits(CURRENT_LATCHES.IR,15,12);
   

   int IR_11 = Get_Bits(CURRENT_LATCHES.IR,11,11) ;

   int J_BITS;

   
   int COND = GetCOND(x);
   
   
   
   switch(COND)
   {
     case 0:
       J_BITS = GetJ(x);
       break;
     case 1:
       J_BITS = (x[J5]<<5) + (x[J4]<<4) + (x[J3]<<3) + (x[J2]<<2) + (CURRENT_LATCHES.READY<<1) + x[J0];
       break;
     case 2:
       J_BITS = (x[J5]<<5) + (x[J4]<<4) + (x[J3]<<3) + (CURRENT_LATCHES.BEN<<2) + (x[J1]<<1) + x[J0];
       break;
     case 3:
       J_BITS = (x[J5]<<5) + (x[J4]<<4) + (x[J3]<<3) + (x[J2]<<2) + (x[J1]<<1) + IR_11;
       break;
       
     default: break;
   }


   int MUX[2];
   MUX[0] = J_BITS;
   MUX[1] = OPCODE;

   int ROW = MUX[GetIRD(x)]; 
   
   
   NEXT_LATCHES.STATE_NUMBER = ROW;
   
   int i=0;
   for(i=0;i<35;i++)
   {
     NEXT_LATCHES.MICROINSTRUCTION[i] = CONTROL_STORE[ROW][i];
   }
   
}


void cycle_memory() 
{   
    NEXT_LATCHES.READY = 0;
    if(GetMIO_EN(x) == 1)
    {
	  // Write
	  if (GetR_W(x)== 1 )
	  {
	      COUNT += 1;
	      if ( COUNT == 4 )
	      {
		  NEXT_LATCHES.READY = 1;   
	      }
	      else if( COUNT == 5)
	      {
		  MEM_WRITE();
		  COUNT = 0;
	      }

	  }
	  
	  //READ
	  if(GetR_W(x) == 0 )
	  {
	      COUNT += 1;
	      if ( COUNT ==4 )	  
	      {
		  NEXT_LATCHES.READY = 1;   

	      }
	      else if( COUNT == 5)
	      {
		NEXT_LATCHES.MDR = MEM_READ();
		COUNT = 0;
	      }

	  }
    }
}


void eval_bus_drivers() {

    vALU = GET_ALU_RESULT();
    

    vPC = CURRENT_LATCHES.PC;
    

    vSHF = GET_SHF_RESULT();
    
    vMDR = CURRENT_LATCHES.MDR;

  
    vMAR = GET_MAR_RESULT();
  
}


void drive_bus() {

    int ALU_BOX[2] = {0,vALU};
    int PC_BOX[2]  = {0,vPC};
    int SHF_BOX[2] = {0,vSHF};
    int MDR_BOX[2] = {0,vMDR};
    int MAR_BOX[2] = {0,vMAR};

    BUS = ALU_BOX[GetGATE_ALU(x)] + 
          PC_BOX[GetGATE_PC(x)]   + 
          SHF_BOX[GetGATE_SHF(x)] + 
          MDR_BOX[GetGATE_MDR(x)] +
          MAR_BOX[GetGATE_MARMUX(x)];
	  
	//  printf("bus%d,\n",BUS);
}


void latch_datapath_values() {

    //MAR    
    int BUS_MAR[2] = { CURRENT_LATCHES.MAR, BUS };
    NEXT_LATCHES.MAR = BUS_MAR[GetLD_MAR(x)];

    //PC
    int BUS_PC[2] = { CURRENT_LATCHES.PC, GET_PC_RESULT() };
    NEXT_LATCHES.PC = BUS_PC[GetLD_PC(x)];

  
    //IR
    int BUS_IR[2] = { CURRENT_LATCHES.IR, BUS};
    NEXT_LATCHES.IR = BUS_IR[GetLD_IR(x)];


  if(GetLD_CC(x)){
   NEXT_LATCHES.N=0;
  NEXT_LATCHES.Z=0;
  NEXT_LATCHES.P=0;
  if ( BUS == 0 )      NEXT_LATCHES.Z=1;
  else if ( BUS & 0x8000 )  NEXT_LATCHES.N=1;
  else                   NEXT_LATCHES.P=1;
  }
    //BEN
    if(GetLD_BEN(x))
    {
      NEXT_LATCHES.BEN = GETBEN();
    }
    
    //MDR
    if(GetMIO_EN(x) == 0)
    {
      	  int MDR_MUX[2];
	  MDR_MUX[0] = CURRENT_LATCHES.MDR;
	  MDR_MUX[1] = BUS;
	  NEXT_LATCHES.MDR = MDR_MUX[GetLD_MDR(x)];
    }

    //REG
    if(GetLD_REG(x))
    {
        int DR_MUX[2];

        DR_MUX[0] = Get_Bits(CURRENT_LATCHES.IR,11,9);
        DR_MUX[1] = 7;

        int DR_ADDR = DR_MUX[GetDRMUX(x)];
        NEXT_LATCHES.REGS[DR_ADDR] = BUS;
    }
}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Routines ///////////////////////////////////

int GET_ALU_RESULT()
{
   int ALU_MUX[4];
    
   int SR1_MUX[2];

   SR1_MUX[0] = Get_Bits(CURRENT_LATCHES.IR,11,9);
   SR1_MUX[1] = Get_Bits(CURRENT_LATCHES.IR,8,6);
   int SR1_ADDR = SR1_MUX[GetSR1MUX(x)];

   int SR2MUX[2];
   int IR_5 = Get_Bits(CURRENT_LATCHES.IR,5,5);
   int SR2_ADDR = Get_Bits(CURRENT_LATCHES.IR,2,0);
   int imm5 = Get_Bits(CURRENT_LATCHES.IR,4,0);
   SR2MUX[0] = CURRENT_LATCHES.REGS[SR2_ADDR];
   SR2MUX[1] = SEXT(imm5,5);

   int A = CURRENT_LATCHES.REGS[SR1_ADDR];
   int B = SR2MUX[IR_5];

    ALU_MUX[0] = A + B;
    ALU_MUX[1] = A & B;
    ALU_MUX[2] = A ^ B;
    ALU_MUX[3] = A;

    return Low16bits(ALU_MUX[GetALUK(x)]);
}

int GET_PC_RESULT()
{
    int MUX[3];

    MUX[0] = CURRENT_LATCHES.PC + 2;
    MUX[1] = BUS;
    MUX[2] = GET_ADDRESS_ADDER();

    return Low16bits(MUX[GetPCMUX(x)]);

}

int GET_MAR_RESULT()
{
    int MUX[2];

    MUX[0] = LSHF( ZEXT(Get_Bits(CURRENT_LATCHES.IR,7,0)),1);
    
    MUX[1] = GET_ADDRESS_ADDER();

    return  Low16bits(MUX[GetMARMUX(x)]);

}

int GET_MDR_RESULT()
{
    return CURRENT_LATCHES.MDR;
}

int GET_SHF_RESULT()
{
    int amount4 = Get_Bits(CURRENT_LATCHES.IR,3,0);
    int IR_5 = Get_Bits(CURRENT_LATCHES.IR,5,5);
    int IR_4 = Get_Bits(CURRENT_LATCHES.IR,4,4);
    int SR_ADDR = Get_Bits(CURRENT_LATCHES.IR,8,6);
    int SR = CURRENT_LATCHES.REGS[SR_ADDR];
    int SR_topbit = SR>>15;
    int SHF_CONTROL = (IR_5<<1) + (IR_4);

    int MUX[4];

    MUX[0] = LSHF(SR,amount4);              // SHFL
    MUX[1] = RSHF(SR,amount4,0);            // RSHFL
    MUX[2] = 0;                             // No this condition
    MUX[3] = RSHF(SR,amount4,SR_topbit);    // RSHFA

    return Low16bits(MUX[SHF_CONTROL]);

}


int GET_ADDRESS_ADDER()
{
    int ADDR_ADDER_IN_1;
    int ADDR_ADDER_IN_2;

    // IN_1
    int R1_MUX[2];

    int BaseR_ADDR = Get_Bits(CURRENT_LATCHES.IR,8,6);
    int BaseR = CURRENT_LATCHES.REGS[BaseR_ADDR];

    R1_MUX[0] = CURRENT_LATCHES.PC;
    R1_MUX[1] = BaseR;

    // IN_2
    int R2_MUX[4];

    int ZERO = 0;
    int offset6 = SEXT(Get_Bits(CURRENT_LATCHES.IR,5,0),6);
    int offset9 = SEXT(Get_Bits(CURRENT_LATCHES.IR,8,0),9);
    int offset11 = SEXT(Get_Bits(CURRENT_LATCHES.IR,10,0),11);

    R2_MUX[0] = ZERO;
    R2_MUX[1] = offset6;
    R2_MUX[2] = offset9;
    R2_MUX[3] = offset11;

    int LSHF1 = LSHF(R2_MUX[GetADDR2MUX(x)],GetLSHF1(x));
    
    ADDR_ADDER_IN_1 = LSHF1;
    ADDR_ADDER_IN_2 = R1_MUX[GetADDR1MUX(x)];

    return Low16bits(ADDR_ADDER_IN_1 + ADDR_ADDER_IN_2);
}

int GETBEN()
{
    int IR_11 = Get_Bits(CURRENT_LATCHES.IR,11,11);
    int IR_10 = Get_Bits(CURRENT_LATCHES.IR,10,10);
    int IR_9  = Get_Bits(CURRENT_LATCHES.IR, 9, 9);
    int IR_15_12 = Get_Bits(CURRENT_LATCHES.IR, 15,12);

    int BEN1 = (IR_11 & CURRENT_LATCHES.N)+
              (IR_10 & CURRENT_LATCHES.Z)+
              (IR_9 & CURRENT_LATCHES.P);
    return BEN1;
}

int MEM_READ()
{
  int addr = CURRENT_LATCHES.MAR;
  int bank=addr&1;
  
  if(GetDATA_SIZE(x)==1)
  {
    return MEMORY[addr>>1][1]<<8 | MEMORY[addr>>1][0];
  }
  if(GetDATA_SIZE(x)==0)
  {
    return Low16bits(SEXT(MEMORY[addr>>1][bank],8));
  }
}

int MEM_WRITE()
{
  int addr = CURRENT_LATCHES.MAR;
  int bank=addr&1;
  
  if(GetDATA_SIZE(x)==0)
  {
    MEMORY[addr>>1][bank]= CURRENT_LATCHES.MDR & 0xFF;
  }
  
  if(GetDATA_SIZE(x)==1)
  {
    MEMORY[addr>>1][1] = (CURRENT_LATCHES.MDR & 0x0000FF00) >> 8;
    MEMORY[addr>>1][0] = CURRENT_LATCHES.MDR & 0xFF;
  }
  return 0;
}
