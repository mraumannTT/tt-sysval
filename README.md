# tt-sysval: Tenstorrent System Validation Suite

Automated hardware validation tool for AI accelerator deployment readiness.

## Purpose

Validates that server/workstation hardware is properly configured and performing at expected levels before installing Tenstorrent AI accelerator cards.

## What It Does

- **Memory Bandwidth Testing** - Ensures DDR4/DDR5 memory is running at full speed
- **CPU Stress Testing** - Validates thermal and performance limits  
- **System Configuration Analysis** - Checks hardware setup and BIOS settings
- **Deployment Readiness Assessment** - Clear pass/fail validation for AI accelerator installation

## Key Features

- Automatic hardware detection (CPU, memory type, channels)
- Memory type-aware validation thresholds
- Server-grade platform support (EPYC, Xeon)
- Detects common BIOS configuration issues
- Comprehensive JSON reporting

## Usage

```bash
# Basic validation
sudo python3 tt-sysval.py

# Detailed report with longer tests
sudo python3 tt-sysval.py -v -d 60 -o report.json
```

## Dependencies

```bash
sudo apt install stress-ng dmidecode python3-psutil
```

## Exit Codes

- **0**: Ready for AI accelerator deployment
- **1**: Has warnings but may be suitable  
- **2**: Failed validation - fix issues before deployment
