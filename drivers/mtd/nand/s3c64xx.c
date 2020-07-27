/*
 * (C) Copyright 2006 DENX Software Engineering
 *
 * Implementation for U-Boot 1.1.6 by Samsung
 *
 * (C) Copyright 2008
 * Guennadi Liakhovetki, DENX Software Engineering, <lg@denx.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#include <nand.h>
#include <linux/mtd/nand.h>

#include <asm/arch-s3c64xx/s3c6410.h>

#include <asm/io.h>
#include <asm/errno.h>

#define MAX_CHIPS	2
static int nand_cs[MAX_CHIPS] = {0, 1};

#ifdef CONFIG_NAND_SPL
#define printf(arg...) do {} while (0)
#endif

/* Nand flash definition values by jsgood */
#ifdef S3C_NAND_DEBUG
/*
 * Function to print out oob buffer for debugging
 * Written by jsgood
 */
static void print_oob(const char *header, struct mtd_info *mtd)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	printf("%s:\t", header);

	for (i = 0; i < 64; i++)
		printf("%02x ", chip->oob_poi[i]);

	printf("\n");
}
#endif /* S3C_NAND_DEBUG */

static void s3c_nand_select_chip(struct mtd_info *mtd, int chip)
{
	int ctrl = readl(NFCONT);

	/*if(chip<0)
		printf("diable cs\n");
	else
		printf("select cs%d\n",chip);	
	*/
	switch (chip) {
	case -1:
		ctrl |= 6;
		break;
	case 0:
		ctrl &= ~2;
		break;
	case 1:
		ctrl &= ~4;
		break;
	default:
		return;
	}

	writel(ctrl, NFCONT);
}

/*
 * Hardware specific access to control-lines function
 * Written by jsgood
 */
static void s3c_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;
	
	if (ctrl & NAND_CTRL_CHANGE) {//ctrl =0x8x
		if (ctrl & NAND_CLE)//ctrl == 0x83	
			this->IO_ADDR_W = (void __iomem *)NFCMMD;
		else if (ctrl & NAND_ALE)
			this->IO_ADDR_W = (void __iomem *)NFADDR;
		else
			this->IO_ADDR_W = (void __iomem *)NFDATA;

		if (ctrl & NAND_NCE)
			s3c_nand_select_chip(mtd, *(int *)this->priv);
		else
			s3c_nand_select_chip(mtd, -1);
	}
	//printf("In s3c6410_hwcontrol.ctrl is 0x%x,cmd is 0x%x\n",ctrl,cmd);
	if (cmd != NAND_CMD_NONE)
		writeb(cmd, this->IO_ADDR_W);
}

/*
 * Function for checking device ready pin
 * Written by jsgood
 */
static int s3c_nand_device_ready(struct mtd_info *mtdinfo)
{
	unsigned int ready = !!(readl(NFSTAT) & NFSTAT_RnB);
	//printf("In s3c6410_dev_ready,dev is %s ready.\n",ready?"":"not");
	return ready;
}

#ifdef CONFIG_SYS_S3C_NAND_HWECC
/*
 * This function is called before encoding ecc codes to ready ecc engine.
 * Written by jsgood
 */
 
#ifdef CONFIG_NAND_BL1_8BIT_ECC
 /***************************************************************
  * jsgood: Temporary 8 Bit H/W ECC supports for BL1 (6410/6430 only)
  ***************************************************************/
 int cur_ecc_mode=0;

// nf8eccerr update NFSTAT[6] = 1 ECCDecDone
static void s3c_nand_wait_enc(void)
{
	while (!(readl(NFSTAT) & NFSTAT_ECCENCDONE)) {}
}
	
/*
 * Function for checking ECCDecDone in NFSTAT
 * Written by jsgood
 */
static void s3c_nand_wait_dec(void)
{
	while (!(readl(NFSTAT) & NFSTAT_ECCDECDONE)) {}
}
  
 static void s3c_nand_wait_ecc_busy_8bit(void)
 {
	 while (readl(NF8ECCERR0) & 0x80000000);
 }
 void s3c_nand_enable_hwecc_8bit(struct mtd_info *mtd, int mode)
{
    u_long nfcont, nfconf;

    cur_ecc_mode = mode;

    //清ECCTypte,设置ECCTpye = 8bit
    nfconf = readl(NFCONF);
    nfconf &= ~(0x3 << 23);
    nfconf |= NFCONF_ECC_8BIT;
    writel(nfconf, NFCONF);
    //解锁MECC
    nfcont = readl(NFCONT);
    nfcont &= ~NFCONT_MECCLOCK;
    //根据读写，控制ECC模块encode or decode
    if (mode == NAND_ECC_WRITE)
        nfcont |= NFCONT_ECC_ENC;	//1 - 编码
    else if (mode == NAND_ECC_READ)
        nfcont &= ~NFCONT_ECC_ENC;	//0 - 解码
    nfcont |= (1<<11); //stop ecc before initlize ecc
    writel(nfcont, NFCONT);
    //初始化MECC和SECC
    nfcont = readl(NFCONT);
    nfcont |= NFCONT_INITECC;
    writel(nfcont, NFCONT);
}
int s3c_nand_calculate_ecc_8bit(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
    u_long nfcont, nfm8ecc0, nfm8ecc1, nfm8ecc2, nfm8ecc3;

    

    if (cur_ecc_mode == NAND_ECC_READ)
    {
	//锁MECC
    	nfcont = readl(NFCONT);
    	nfcont |= NFCONT_MECCLOCK;
    	writel(nfcont, NFCONT);

	//printf("NAND:wait ecc decode\n");
	s3c_nand_wait_dec();
	//printf("NAND:ecc decode finished\n");
	
    }  
    else {
	//printf("NAND:wait ecc encode\n");
        s3c_nand_wait_enc();
	//printf("NAND:ecc encode finished\n");

        //锁MECC
    	nfcont = readl(NFCONT);
	//printf("debug 1\n");
    	nfcont |= NFCONT_MECCLOCK;
    	writel(nfcont, NFCONT);
	//printf("debug 2\n");
        nfm8ecc0 = NFM8ECC0;
        nfm8ecc1 = NFM8ECC1;
        nfm8ecc2 = NFM8ECC2;
        nfm8ecc3 = NFM8ECC3;
	//printf("debug 3\n");
        ecc_code[0] = nfm8ecc0 & 0xff;
        ecc_code[1] = (nfm8ecc0 >> 8) & 0xff;
        ecc_code[2] = (nfm8ecc0 >> 16) & 0xff;
        ecc_code[3] = (nfm8ecc0 >> 24) & 0xff;
        ecc_code[4] = nfm8ecc1 & 0xff;
        ecc_code[5] = (nfm8ecc1 >> 8) & 0xff;
        ecc_code[6] = (nfm8ecc1 >> 16) & 0xff;
        ecc_code[7] = (nfm8ecc1 >> 24) & 0xff;
        ecc_code[8] = nfm8ecc2 & 0xff;
        ecc_code[9] = (nfm8ecc2 >> 8) & 0xff;
        ecc_code[10] = (nfm8ecc2 >> 16) & 0xff;
        ecc_code[11] = (nfm8ecc2 >> 24) & 0xff;
        ecc_code[12] = nfm8ecc3 & 0xff;
	//printf("debug 4\n");
    }

    return 0;
}

int s3c_nand_correct_data_8bit(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
    int ret = -1;
    u_long nf8eccerr0, nf8eccerr1, nf8eccerr2, nfmlc8bitpt0, nfmlc8bitpt1;
    u_char err_type;

    //s3c_nand_wait_ecc_busy_8bit();

    nf8eccerr0 = NF8ECCERR0;
    nf8eccerr1 = NF8ECCERR1;
    nf8eccerr2 = NF8ECCERR2;
    nfmlc8bitpt0 = NFMLC8BITPT0;
    nfmlc8bitpt1 = NFMLC8BITPT1;

    err_type = (nf8eccerr0 >> 25) & 0xf;

    /* No error, If free page (all 0xff) */
    if ((nf8eccerr0 >> 29) & 0x1)
        err_type = 0;

    switch (err_type)
    {
    case 8: /* 8 bit error (Correctable) */
        dat[(nf8eccerr2 >> 22) & 0x3ff] ^= ((nfmlc8bitpt1 >> 24) & 0xff);
        printk("s3c-nand: %d bit(s) error detected, corrected successfully\n", err_type);

    case 7: /* 7 bit error (Correctable) */
        dat[(nf8eccerr2 >> 11) & 0x3ff] ^= ((nfmlc8bitpt1 >> 16) & 0xff);
        printk("s3c-nand: %d bit(s) error detected, corrected successfully\n", err_type);

    case 6: /* 6 bit error (Correctable) */
        dat[nf8eccerr2 & 0x3ff] ^= ((nfmlc8bitpt1 >> 8) & 0xff);
        printk("s3c-nand: %d bit(s) error detected, corrected successfully\n", err_type);

    case 5: /* 5 bit error (Correctable) */
        dat[(nf8eccerr1 >> 22) & 0x3ff] ^= (nfmlc8bitpt1 & 0xff);
        printk("s3c-nand: %d bit(s) error detected, corrected successfully\n", err_type);

    case 4: /* 4 bit error (Correctable) */
        dat[(nf8eccerr1 >> 11) & 0x3ff] ^= ((nfmlc8bitpt0 >> 24) & 0xff);
        printk("s3c-nand: %d bit(s) error detected, corrected successfully\n", err_type);

    case 3: /* 3 bit error (Correctable) */
        dat[nf8eccerr1 & 0x3ff] ^= ((nfmlc8bitpt0 >> 16) & 0xff);
        printk("s3c-nand: %d bit(s) error detected, corrected successfully\n", err_type);

    case 2: /* 2 bit error (Correctable) */
        dat[(nf8eccerr0 >> 15) & 0x3ff] ^= ((nfmlc8bitpt0 >> 8) & 0xff);
        printk("s3c-nand: %d bit(s) error detected, corrected successfully\n", err_type);


    case 1: /* 1 bit error (Correctable) */
        printk("s3c-nand: %d bit(s) error detected, corrected successfully\n", err_type);
        dat[nf8eccerr0 & 0x3ff] ^= (nfmlc8bitpt0 & 0xff);
        ret = err_type;
        break;

    case 0: /* No error */
	//printf("s3c-nand: not exist bit error.\n");
        ret = 0;
        break;
    default:
	break;
	//printf("s3c-nand: uncorrectalbe errors.\n");
    }

    return ret;
}
/*
使用8bit ecc
1.设置msglength = 0 (512-byte date per 13-byte ecc codes)
2.使能8bit ecc , set ECCType = 01,每写入512byte数据ecc模块将产生13byte的ecc codes
  在清除MainECClock(NFCONT[7] = 0,'unlock')后，InitMECC(NFCONT[5] = 1),让ecc模块开始工作
  MainECClock(NFCONT[7])控制是否生成ecc code
3.Write data
4.写完512byte的数据（不包括spare area data），校验码自动更新在 NF8MECC0, NFMECC1, NF8MECC2, NF8MECC3
  需要检查encoding done 在NFSTAT中，之后设置 MainECClock = '1',lock.
5.生成spare area的ecc，设置msglength = 1 (24-byte date per 13-byte ecc codes)
  初始化ecc module,在清除MainECClock(NFCONT[7] = 0,'unlock')后，InitMECC(NFCONT[5] = 1)
*/
void s3c_nand_write_page_8bit(struct mtd_info *mtd, struct nand_chip *chip,
                              const uint8_t *buf)
{
    uint32_t *mecc_pos = chip->ecc.layout->eccpos;
    int i, eccsize = 512;
    int eccbytes = 13;
    int eccsteps = mtd->writesize / eccsize; 	// 4096/512 = 8
    uint8_t *ecc_calc = chip->buffers->ecccalc; //ecc_buf
    uint8_t *p = buf;				//dat_buf
//i+=13 , p+= 512
    for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
	//printf("nand write page dubug 1\n");
        s3c_nand_enable_hwecc_8bit(mtd, NAND_ECC_WRITE);
	//printf("nand write page dubug 2\n");
        chip->write_buf(mtd, p, eccsize);
	//printf("nand write page dubug 3\n");
        s3c_nand_calculate_ecc_8bit(mtd, p, &ecc_calc[i]);//将ecc码存入ecc_calc
	//printf("nand write page dubug 4\n");
    }
	
    for (i = 0; i < eccbytes * (mtd->writesize / eccsize); i++)
        chip->oob_poi[mecc_pos[i]] = ecc_calc[i];
    chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
}

int s3c_nand_read_page_8bit(struct mtd_info *mtd, struct nand_chip *chip,
                            uint8_t *buf)
{
    int i, stat, eccsize = 512,j,k;
    int eccbytes = 13;
    int eccsteps = mtd->writesize / eccsize;// = 8
    int col = 0;
    uint32_t *mecc_pos = chip->ecc.layout->eccpos;
    uint8_t *p = buf;

    /* Step1: read whole oob */
    col = mtd->writesize;
    chip->cmdfunc(mtd, NAND_CMD_RNDOUT, col, -1);
    chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);
/*	
	for(j=0;j<104;j++)
	{
		if((j%8)==0)
			printf("\n");

		printf(" %2x ",*(chip->oob_poi+mecc_pos[j]));
	}
*/   
    col = 0;
    for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
        chip->cmdfunc(mtd, NAND_CMD_RNDOUT, col, -1);
        s3c_nand_enable_hwecc_8bit(mtd, NAND_ECC_READ);
        chip->read_buf(mtd, p, eccsize);
	
        chip->write_buf(mtd, chip->oob_poi + mecc_pos[0] + (((mtd->writesize / eccsize) - eccsteps) * eccbytes), eccbytes);
	
        s3c_nand_calculate_ecc_8bit(mtd, 0, 0);

      	stat = s3c_nand_correct_data_8bit(mtd, p, 0, 0);

    	if (stat == -1)
    	mtd->ecc_stats.failed++;
        col = eccsize * ((mtd->writesize / eccsize) + 1 - eccsteps);
    }

    	


    return 0;
}

/********************************************************/
#endif

static void s3c_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	u_long nfcont, nfconf;

	/*
	 * The original driver used 4-bit ECC for "new" MLC chips, i.e., for
	 * those with non-zero ID[3][3:2], which anyway only holds for ST
	 * (Numonyx) chips
	 */

	nfconf = readl(NFCONF) & ~NFCONF_ECC_4BIT;
    
	writel(nfconf, NFCONF);

	/* Initialize & unlock */
	nfcont = readl(NFCONT);
	nfcont |= NFCONT_INITECC;
	nfcont &= ~NFCONT_MECCLOCK;

	if (mode == NAND_ECC_WRITE)
		nfcont |= NFCONT_ECC_ENC;
	else if (mode == NAND_ECC_READ)
		nfcont &= ~NFCONT_ECC_ENC;

	writel(nfcont, NFCONT);
}

/*
 * This function is called immediately after encoding ecc codes.
 * This function returns encoded ecc codes.
 * Written by jsgood
 */
static int s3c_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				  u_char *ecc_code)
{
	u_long nfcont, nfmecc0;
	/* Lock */
	nfcont = readl(NFCONT);
	nfcont |= NFCONT_MECCLOCK;
	writel(nfcont, NFCONT);

	nfmecc0 = readl(NFMECC0);

	ecc_code[0] = nfmecc0 & 0xff;
	ecc_code[1] = (nfmecc0 >> 8) & 0xff;
	ecc_code[2] = (nfmecc0 >> 16) & 0xff;
	ecc_code[3] = (nfmecc0 >> 24) & 0xff;
    
	return 0;
}

/*
 * This function determines whether read data is good or not.
 * If SLC, must write ecc codes to controller before reading status bit.
 * If MLC, status bit is already set, so only reading is needed.
 * If status bit is good, return 0.
 * If correctable errors occured, do that.
 * If uncorrectable errors occured, return -1.
 * Written by jsgood
 */
static int s3c_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				 u_char *read_ecc, u_char *calc_ecc)
{
	int ret = -1;
	u_long nfestat0, nfmeccdata0, nfmeccdata1, err_byte_addr;
	u_char err_type, repaired;

	/* SLC: Write ecc to compare */
	nfmeccdata0 = (calc_ecc[1] << 16) | calc_ecc[0];
	nfmeccdata1 = (calc_ecc[3] << 16) | calc_ecc[2];
	writel(nfmeccdata0, NFMECCDATA0);
	writel(nfmeccdata1, NFMECCDATA1);

	/* Read ecc status */
	nfestat0 = readl(NFESTAT0);
	err_type = nfestat0 & 0x3;

	switch (err_type) {
	case 0: /* No error */
		ret = 0;
		break;

	case 1:
		/*
		 * 1 bit error (Correctable)
		 * (nfestat0 >> 7) & 0x7ff	:error byte number
		 * (nfestat0 >> 4) & 0x7	:error bit number
		 */
		err_byte_addr = (nfestat0 >> 7) & 0x7ff;
		repaired = dat[err_byte_addr] ^ (1 << ((nfestat0 >> 4) & 0x7));

		printf("S3C NAND: 1 bit error detected at byte %ld. "
		       "Correcting from 0x%02x to 0x%02x...OK\n",
		       err_byte_addr, dat[err_byte_addr], repaired);

		dat[err_byte_addr] = repaired;

		ret = 1;
		break;

	case 2: /* Multiple error */
	case 3: /* ECC area error */
		printf("S3C NAND: ECC uncorrectable error detected. "
		       "Not correctable.\n");
		ret = -1;
		break;
	}

	return ret;
}
#endif /* CONFIG_SYS_S3C_NAND_HWECC */

/*
 * Board-specific NAND initialization. The following members of the
 * argument are board-specific (per include/linux/mtd/nand.h):
 * - IO_ADDR_R?: address to read the 8 I/O lines of the flash device
 * - IO_ADDR_W?: address to write the 8 I/O lines of the flash device
 * - hwcontrol: hardwarespecific function for accesing control-lines
 * - dev_ready: hardwarespecific function for  accesing device ready/busy line
 * - enable_hwecc?: function to enable (reset)  hardware ecc generator. Must
 *   only be provided if a hardware ECC is available
 * - eccmode: mode of ecc, see defines
 * - chip_delay: chip dependent delay for transfering data from array to
 *   read regs (tR)
 * - options: various chip options. They can partly be set to inform
 *   nand_scan about special functionality. See the defines for further
 *   explanation
 * Members with a "?" were not set in the merged testing-NAND branch,
 * so they are not set here either.
 */
int board_nand_init(struct nand_chip *nand)
{
	static int chip_n;
	unsigned int cfg;
	unsigned int tacls,twrph0,twrph1;
	
	if (chip_n >= MAX_CHIPS)
		return -ENODEV;

	/* initialize hardware */
#if defined(CONFIG_S3C6410_CUSTOM_NAND_TIMING)
	tacls  = CONFIG_S3C6410_TACLS;
	twrph0 = CONFIG_S3C6410_TWRPH0;
	twrph1 =  CONFIG_S3C6410_TWRPH1;

	cfg = readl(NFCONF); 
	cfg &= ~(tacls << 12);
	cfg &= ~(twrph0 << 8);
	cfg &= ~(twrph1 << 4);
	cfg |= (tacls << 12);
	cfg |= (twrph0 << 8);
	cfg |= (twrph1 << 4);
	writel(cfg,NFCONF);

	cfg = readl(NFCONT); 
	cfg |= 0x7;
	writel(cfg,NFCONT);
#endif
	//关软件锁 使能NF
	NFCONT_REG = (NFCONT_REG & ~NFCONT_WP) | NFCONT_ENABLE | 0x6;

	nand->IO_ADDR_R		= (void __iomem *)NFDATA;
	nand->IO_ADDR_W		= (void __iomem *)NFDATA;
	nand->cmd_ctrl		= s3c_nand_hwcontrol;
	nand->dev_ready		= s3c_nand_device_ready;
	nand->select_chip	= s3c_nand_select_chip;
	nand->options		= 0;
#ifdef CONFIG_NAND_SPL
	nand->read_byte		= nand_read_byte;
	nand->write_buf		= nand_write_buf;
	nand->read_buf		= nand_read_buf;
#endif



#ifdef CONFIG_SYS_S3C_NAND_HWECC
    #ifdef CONFIG_NAND_BL1_8BIT_ECC
	printf("USE HWECC 8BIT\n");//zxd
	nand->ecc.hwctl		= s3c_nand_enable_hwecc_8bit;
	nand->ecc.calculate	= s3c_nand_calculate_ecc_8bit;
	nand->ecc.correct	= s3c_nand_correct_data_8bit;
	nand->ecc.read_page 	= s3c_nand_read_page_8bit;
    	nand->ecc.write_page 	= s3c_nand_write_page_8bit;
    #else
	printf("USE HWECC Default\n");//zxd
	nand->ecc.hwctl		= s3c_nand_enable_hwecc;
	nand->ecc.calculate	= s3c_nand_calculate_ecc;
	nand->ecc.correct	= s3c_nand_correct_data;
    #endif
	/*
	 * If you get more than 1 NAND-chip with different page-sizes on the
	 * board one day, it will get more complicated...
	 */
	nand->ecc.mode		= NAND_ECC_HW;
	nand->ecc.size		= CONFIG_SYS_NAND_ECCSIZE;
	nand->ecc.bytes		= CONFIG_SYS_NAND_ECCBYTES;
    	printf("ECC Size:%d ECC Bytes:%d\n",nand->ecc.size,nand->ecc.bytes);//zxd	
#else
	nand->ecc.mode		= NAND_ECC_SOFT;
#endif /* ! CONFIG_SYS_S3C_NAND_HWECC */

	nand->priv		= nand_cs + chip_n++;

	return 0;
}


