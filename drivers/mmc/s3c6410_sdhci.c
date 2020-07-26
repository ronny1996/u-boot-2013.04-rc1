/*
        decription : Samsung S3C6410 MMC Driver
        function   :
        author	   : Ronny
        date	   : 2018.8.21
*/

/*
                                文件系统应用层(common/cmd_fat.c)
                                                |
                                FAT文件系统(Fs/fat/fat.c)
                                                |
SD/MMC应用层(common/cmd_mmc.c) 	 SD/MMC驱动通用层(drivers/mmc/mmc.c)
MMC环境变量(common/env_mmc.c)
                                                |
                                SD/MMC主机控制器(drivers/mmc/sdhci.c)
*/

#include <asm/arch/s3c6410.h>
#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <sdhci.h>

#ifdef CONFIG_MMC_CHANNEL
#define MMC_CHANNEL CONFIG_MMC_CHANNEL
#else
#define MMC_CHANNEL 0
#endif

#define ELFIN_HSMMC_BASE 0x7C200000
#define MMC_REGS_BASE (ELFIN_HSMMC_BASE + 0x100000 * MMC_CHANNEL)

static void sdhc_set_gpio() {
  u32 reg;
#if (MMC_CHANNEL == 0)
  reg = readl(GPGCON) & 0xF0000000;
  writel(reg | 0x02222222, GPGCON);

  reg = readl(GPGPUD) & 0xFFFFF000;
  writel(reg, GPGPUD);
#elif (MMC_CHANNEL == 1)
  writel(0x00222222, GPHCON0);
  writel(0x00000000, GPHCON1);

  reg = readl(GPHPUD) & 0xFFFFF000;
  writel(reg, GPHPUD);
#else
  printf("#####err : SDMMC channel is not defined!\n");
#endif
}

int s3c_sdhci_init(u32 regbase, u32 max_clk, u32 min_clk, u32 quirks) {
  struct sdhci_host *host = NULL;

  host = (struct sdhci_host *)malloc(sizeof(struct sdhci_host));
  if (!host) {
    printf("#####err : sdhci_host malloc fails!\n");
    return 1;
  }
  sdhc_set_gpio();
  host->name = "Samsung Host Controller";
  host->ioaddr = (void *)regbase;
  host->quirks = quirks;
  host->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
  if (quirks & SDHCI_QUIRK_REG32_RW) {
    host->version = sdhci_readl(host, SDHCI_HOST_VERSION - 2) >> 16;
  } else {
    host->version = sdhci_readl(host, SDHCI_HOST_VERSION);
  }

  add_sdhci(host, max_clk, min_clk);

  return 0;
}
int board_mmc_init(bd_t *bis) {
  // return s3c_sdhci_init(MMC_REGS_BASE,52000000,400000,0);
  return s3c_sdhci_init(MMC_REGS_BASE, 52000000, 400000, SDHCI_QUIRK_REG32_RW);
}
