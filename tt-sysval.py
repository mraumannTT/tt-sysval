#!/usr/bin/env python3
"""
tt-sysval: Tenstorrent System Validation Suite
Hardware validation for AI accelerator card deployment readiness

Copyright 2025 Tenstorrent Inc.
Licensed under the Apache License, Version 2.0
"""

import os
import sys
import time
import subprocess
import json
import psutil
import re
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
import argparse
import shutil

__version__ = "1.1.0"
__author__ = "Tenstorrent Inc."

class TenstorrentSystemValidator:
    def __init__(self, verbose=False, duration=30):
        self.verbose = verbose
        self.test_duration = duration
        self.results = {
            'timestamp': datetime.now().isoformat(),
            'system_info': {},
            'tests': {},
            'validation_summary': {}
        }
        
        # Memory type-specific thresholds
        self.memory_thresholds = {
            'DDR5': {'min_bandwidth_gbps': 50, 'efficiency_critical': 10, 'efficiency_warning': 25},
            'DDR4': {'min_bandwidth_gbps': 20, 'efficiency_critical': 20, 'efficiency_warning': 40},
            'DDR3': {'min_bandwidth_gbps': 15, 'efficiency_critical': 25, 'efficiency_warning': 45},
            'Unknown': {'min_bandwidth_gbps': 10, 'efficiency_critical': 15, 'efficiency_warning': 25}
        }
        
        self.general_thresholds = {
            'max_cpu_temp': 85,
            'max_load_average': 2.0,
            'min_free_memory_pct': 10,
        }

    def log(self, message):
        if self.verbose:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")

    def run_command(self, cmd, timeout=10, check_return=True):
        """Unified command execution with error handling"""
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            if check_return and result.returncode != 0:
                return None
            return result
        except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
            return None

    def check_dependencies(self):
        """Check if required tools are available"""
        self.log("Checking dependencies...")
        
        if not shutil.which('stress-ng'):
            print("ERROR: stress-ng not found. Install with:")
            print("  Ubuntu/Debian: sudo apt-get install stress-ng")
            print("  RHEL/CentOS: sudo yum install stress-ng")
            print("  Fedora: sudo dnf install stress-ng")
            return False
        
        # Warn about desktop environment
        desktop_env = os.environ.get('DESKTOP_SESSION') or os.environ.get('XDG_CURRENT_DESKTOP')
        if desktop_env and self.verbose:
            print(f"⚠️  Running under desktop environment ({desktop_env})")
            print("   For production validation, consider console mode.\n")
        
        return True

    def gather_system_info(self):
        """Collect comprehensive system information"""
        self.log("Gathering system information...")
        
        info = {
            'cpu_count': psutil.cpu_count(logical=True),
            'cpu_physical_cores': psutil.cpu_count(logical=False),
            'memory_total_gb': round(psutil.virtual_memory().total / (1024**3), 2),
            'python_version': sys.version.split()[0],
            'platform': dict(zip(['system', 'node', 'release', 'version', 'machine'], os.uname()))
        }
        
        # Get CPU details
        lscpu_result = self.run_command(['lscpu'])
        if lscpu_result:
            cpu_info = {}
            for line in lscpu_result.stdout.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip().lower().replace(' ', '_')
                    if key in ['model_name', 'cpu_family', 'cpu_mhz', 'cache_size']:
                        cpu_info[key] = value.strip()
            info['cpu_details'] = cpu_info

        # Get memory configuration
        self.log("Detecting memory configuration...")
        memory_info = self.parse_memory_config()
        if memory_info:
            info.update(memory_info)
            
            # Detect actual memory speed vs rated speed
            if info.get('memory_speed_mhz', 0) == 0:
                detected_speed = self.detect_memory_speed_alternative()
                if detected_speed > 0:
                    info['memory_speed_mhz'] = detected_speed
                else:
                    info['memory_speed_mhz'] = self.estimate_memory_speed(info)
            
            # Correct channel estimation for server CPUs
            self.correct_memory_channels(info)

        # Get stress-ng version and Tenstorrent cards
        stress_result = self.run_command(['stress-ng', '--version'])
        info['stress_ng_version'] = stress_result.stdout.strip().split('\n')[0] if stress_result else 'Unknown'
        
        lspci_result = self.run_command(['lspci'])
        if lspci_result:
            tt_cards = [line.strip() for line in lspci_result.stdout.split('\n')
                       if any(kw in line.lower() for kw in ['tenstorrent', 'grayskull', 'wormhole', 'blackhole'])]
            info['tenstorrent_cards'] = tt_cards

        self.results['system_info'] = info
        return info

    def parse_memory_config(self):
        """Parse memory configuration from dmidecode"""
        dmidecode_result = self.run_command(['dmidecode', '--type', 'memory'], check_return=False)
        if not dmidecode_result or dmidecode_result.returncode != 0:
            self.log("dmidecode failed - limited memory detection")
            return {'memory_type': 'Unknown', 'memory_speed_mhz': 0, 'memory_channels': 4, 'populated_dimms': 0}

        memory_info = {'memory_channels': 0, 'memory_speed_mhz': 0, 'memory_type': 'Unknown', 'populated_dimms': 0}
        current_dimm = {}
        
        if self.verbose:
            print("\nParsing memory configuration from dmidecode...")

        for line in dmidecode_result.stdout.split('\n'):
            line = line.strip()
            if 'Memory Device' in line:
                if current_dimm.get('size', 0) > 0:
                    memory_info['populated_dimms'] += 1
                    if self.verbose:
                        print(f"  Found DIMM: {current_dimm.get('size', 0)}GB {current_dimm.get('type', 'Unknown')} @ {current_dimm.get('speed', 0)}MHz")
                current_dimm = {}
            elif line.startswith('Size:') and 'GB' in line and 'No Module Installed' not in line:
                try:
                    current_dimm['size'] = int(line.split(':', 1)[1].strip().split()[0])
                except:
                    pass
            elif line.startswith('Type:'):
                mem_type = line.split(':', 1)[1].strip()
                if mem_type not in ['Unknown', '<OUT OF SPEC>', 'Other']:
                    current_dimm['type'] = mem_type
                    if memory_info['memory_type'] == 'Unknown':
                        memory_info['memory_type'] = mem_type
            elif line.startswith(('Speed:', 'Configured Memory Speed:', 'Configured Clock Speed:')):
                speed_str = line.split(':', 1)[1].strip()
                if 'MHz' in speed_str and 'Unknown' not in speed_str:
                    try:
                        speed_mhz = int(speed_str.split()[0])
                        current_dimm['speed'] = speed_mhz
                        if speed_mhz > memory_info['memory_speed_mhz']:
                            memory_info['memory_speed_mhz'] = speed_mhz
                    except:
                        pass

        # Handle last DIMM
        if current_dimm.get('size', 0) > 0:
            memory_info['populated_dimms'] += 1
            if self.verbose:
                print(f"  Found DIMM: {current_dimm.get('size', 0)}GB {current_dimm.get('type', 'Unknown')} @ {current_dimm.get('speed', 0)}MHz")

        # Basic channel estimation (will be corrected for server CPUs)
        populated = memory_info['populated_dimms']
        memory_info['memory_channels'] = min(max(populated // 2, 1), 4) if populated <= 8 else 6

        if self.verbose:
            print(f"Memory configuration summary (preliminary):")
            print(f"  Type: {memory_info['memory_type']}")
            print(f"  Speed: {memory_info['memory_speed_mhz']} MHz")
            print(f"  Populated DIMMs: {memory_info['populated_dimms']}")
            print(f"  Estimated Channels: {memory_info['memory_channels']} (will be corrected after CPU detection)")

        return memory_info

    def detect_memory_speed_alternative(self):
        """Detect actual vs rated memory speed"""
        dmidecode_result = self.run_command(['dmidecode', '-t', '17'])
        if not dmidecode_result:
            return 0

        configured_speed = rated_speed = 0
        for line in dmidecode_result.stdout.split('\n'):
            if ('Configured Memory Speed:' in line or 'Configured Clock Speed:' in line) and 'Unknown' not in line:
                match = re.search(r'(\d+)\s*(?:MT/s|MHz)', line)
                if match:
                    configured_speed = max(configured_speed, int(match.group(1)))
            elif line.strip().startswith('Speed:') and 'Unknown' not in line:
                match = re.search(r'(\d+)\s*(?:MT/s|MHz)', line)
                if match:
                    rated_speed = max(rated_speed, int(match.group(1)))

        if configured_speed > 0 and rated_speed > 0 and self.verbose:
            print(f"  Memory Speed Analysis:")
            print(f"    Rated Speed: {rated_speed} MT/s (DIMM capability)")
            print(f"    Configured Speed: {configured_speed} MT/s (actual running speed)")
            if configured_speed < rated_speed:
                print(f"    ⚠️  Memory running {rated_speed - configured_speed} MT/s below rated speed!")
                print(f"    💡 Enable XMP/DOCP in BIOS to unlock full performance")

        return configured_speed if configured_speed > 0 else rated_speed

    def estimate_memory_speed(self, system_info):
        """Estimate memory speed based on CPU and memory type"""
        memory_type = system_info.get('memory_type', 'Unknown')
        cpu_model = system_info.get('cpu_details', {}).get('model_name', '').lower()
        
        # CPU-specific defaults
        if 'epyc' in cpu_model:
            return 4800 if memory_type.startswith('DDR5') else 3200
        elif 'ryzen' in cpu_model:
            return 3600 if '5000' in cpu_model else 3200
        elif 'intel' in cpu_model and any(gen in cpu_model for gen in ['10th', '11th', '12th', '13th']):
            return 3200
        
        # Memory type defaults
        return {'DDR5': 4800, 'DDR4': 2400, 'DDR3': 1600}.get(memory_type[:4], 2400)

    def correct_memory_channels(self, system_info):
        """Correct memory channel estimation for server CPUs"""
        cpu_model = system_info.get('cpu_details', {}).get('model_name', '').lower()
        populated_dimms = system_info.get('populated_dimms', 0)
        current_channels = system_info.get('memory_channels', 0)
        
        is_epyc = 'epyc' in cpu_model
        is_server_class = any(keyword in cpu_model for keyword in ['epyc', 'xeon', 'threadripper'])
        
        if self.verbose:
            print(f"🔧 Correcting channel estimation:")
            print(f"   CPU: '{cpu_model}' (EPYC: {is_epyc}, Server: {is_server_class})")
            print(f"   DIMMs: {populated_dimms}, Current channels: {current_channels}")

        # Correct for server systems
        if is_epyc and populated_dimms == 8:
            system_info['memory_channels'] = 8
            if self.verbose:
                print(f"   ✅ Corrected to 8 channels for EPYC with 8 DIMMs")
        elif is_server_class and populated_dimms == 12:
            system_info['memory_channels'] = 6
            if self.verbose:
                print(f"   ✅ Corrected to 6 channels for server with 12 DIMMs")
        elif self.verbose:
            print(f"   ➡️  No correction needed ({current_channels} channels)")

    def collect_system_metrics(self):
        """Collect current system metrics"""
        try:
            metrics = {
                'cpu_percent': psutil.cpu_percent(interval=None),
                'memory': psutil.virtual_memory()._asdict(),
                'load_avg': os.getloadavg(),
            }
            
            # Get CPU temperature if available
            try:
                temps = psutil.sensors_temperatures()
                if temps:
                    cpu_temps = [entry.current for name, entries in temps.items() 
                               if 'cpu' in name.lower() or 'core' in name.lower() 
                               for entry in entries if entry.current]
                    if cpu_temps:
                        metrics['max_cpu_temp'] = max(cpu_temps)
            except:
                pass
            
            return metrics
        except Exception as e:
            return {'error': str(e)}

    def parse_stress_ng_output(self, stdout, stderr):
        """Parse stress-ng output for bandwidth metrics"""
        results = {'metrics': {}, 'errors': [], 'warnings': []}
        combined_output = stdout + '\n' + stderr
        
        for line in combined_output.split('\n'):
            # Parse different bandwidth output formats
            patterns = [
                (r'memory rate:\s*(\d+\.?\d*)\s*MB\s*read/sec,\s*(\d+\.?\d*)\s*MB\s*write/sec', 'separate'),
                (r'memory rate:\s*(\d+\.?\d*)\s*MB/sec', 'combined'),
                (r'stream\s+(\d+\.?\d*)\s+memory rate \(MB per sec\)', 'summary'),
                (r'(\d+\.?\d*)\s+bogo\s+ops/s', 'bogo'),
                (r'(\d+\.?\d*)\s+GB/se?c?', 'gbps'),
            ]
            
            for pattern, ptype in patterns:
                match = re.search(pattern, line)
                if match:
                    if ptype == 'separate':
                        read_mbps, write_mbps = float(match.group(1)), float(match.group(2))
                        results['metrics'].update({
                            'read_bandwidth_mbps': read_mbps,
                            'write_bandwidth_mbps': write_mbps,
                            'total_bandwidth_mbps': read_mbps + write_mbps,
                            'throughput_mbps': read_mbps + write_mbps
                        })
                    elif ptype in ['combined', 'summary']:
                        combined_mbps = float(match.group(1))
                        results['metrics'].update({
                            'total_bandwidth_mbps': combined_mbps,
                            'throughput_mbps': combined_mbps,
                            'read_bandwidth_mbps': combined_mbps * 0.6,
                            'write_bandwidth_mbps': combined_mbps * 0.4
                        })
                    elif ptype == 'bogo':
                        results['metrics']['bogo_ops_per_sec'] = float(match.group(1))
                    elif ptype == 'gbps':
                        results['metrics']['throughput_mbps'] = float(match.group(1)) * 1000
                    break
        
        # Look for errors in stderr
        for line in stderr.split('\n'):
            if 'error' in line.lower() and 'stress-ng:' in line:
                results['errors'].append(line.strip())
            elif 'warning' in line.lower() and 'stress-ng:' in line:
                results['warnings'].append(line.strip())
        
        return results

    def run_stress_test(self, test_name, stress_args):
        """Run stress-ng test with system monitoring"""
        self.log(f"Running {test_name} for {self.test_duration} seconds...")
        
        cmd = ['stress-ng'] + stress_args + [
            '--timeout', f'{self.test_duration}s',
            '--metrics-brief', '--verify'
        ]
        if self.verbose:
            cmd.append('--verbose')
        
        baseline = self.collect_system_metrics()
        metrics_data = {'samples': [], 'baseline': baseline}
        stop_collection = False
        
        def collect_metrics():
            while not stop_collection:
                sample = self.collect_system_metrics()
                sample['timestamp'] = time.time()
                metrics_data['samples'].append(sample)
                time.sleep(1)
        
        try:
            with ThreadPoolExecutor(max_workers=1) as executor:
                metrics_future = executor.submit(collect_metrics)
                start_time = time.time()
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=self.test_duration + 30)
                end_time = time.time()
                stop_collection = True
                metrics_future.result(timeout=5)
            
            metrics_analysis = self.analyze_metrics(metrics_data)
            
            return {
                'status': 'PASS' if result.returncode == 0 else 'FAIL',
                'duration': round(end_time - start_time, 2),
                'system_metrics': metrics_analysis,
                'validation': self.validate_test_results(metrics_analysis),
                'error': f"stress-ng exited with code {result.returncode}" if result.returncode != 0 else None,
                'stderr': result.stderr if result.returncode != 0 and result.stderr else None
            }
        
        except subprocess.TimeoutExpired:
            stop_collection = True
            return {'status': 'TIMEOUT', 'error': f'Test timed out after {self.test_duration + 30} seconds'}
        except Exception as e:
            stop_collection = True
            return {'status': 'ERROR', 'error': str(e)}

    def analyze_metrics(self, metrics_data):
        """Analyze collected system metrics"""
        if not metrics_data['samples']:
            return {'error': 'No metrics samples collected'}
        
        samples = metrics_data['samples']
        
        def extract_values(key_path):
            values = []
            for s in samples:
                try:
                    value = s
                    for key in key_path.split('.'):
                        value = value[key]
                    values.append(value)
                except (KeyError, TypeError):
                    pass
            return values
        
        cpu_usage = extract_values('cpu_percent')
        memory_usage = extract_values('memory.percent')
        load_avgs = [s.get('load_avg', [0])[0] for s in samples if 'load_avg' in s]
        max_temps = extract_values('max_cpu_temp')
        
        analysis = {
            'samples_collected': len(samples),
            'cpu_usage': {'avg': round(sum(cpu_usage) / len(cpu_usage), 2) if cpu_usage else 0, 'max': max(cpu_usage) if cpu_usage else 0},
            'memory_usage': {'avg': round(sum(memory_usage) / len(memory_usage), 2) if memory_usage else 0, 'max': max(memory_usage) if memory_usage else 0},
            'load_average': {'avg': round(sum(load_avgs) / len(load_avgs), 2) if load_avgs else 0, 'max': max(load_avgs) if load_avgs else 0}
        }
        
        if max_temps:
            analysis['temperature'] = {'max': max(max_temps), 'avg': round(sum(max_temps) / len(max_temps), 2)}
        
        return analysis

    def validate_test_results(self, metrics_analysis):
        """Validate test results against deployment criteria"""
        validation = {'passed': True, 'warnings': [], 'failures': []}
        
        # Temperature check
        if 'temperature' in metrics_analysis:
            max_temp = metrics_analysis['temperature']['max']
            if max_temp > self.general_thresholds['max_cpu_temp']:
                validation['failures'].append(f"CPU temperature too high: {max_temp}°C > {self.general_thresholds['max_cpu_temp']}°C")
                validation['passed'] = False
            elif max_temp > self.general_thresholds['max_cpu_temp'] - 10:
                validation['warnings'].append(f"CPU temperature concerning: {max_temp}°C")
        
        # Load average check
        max_load = metrics_analysis.get('load_average', {}).get('max', 0)
        cpu_count = psutil.cpu_count()
        if max_load > cpu_count * self.general_thresholds['max_load_average']:
            validation['failures'].append(f"System overloaded: load {max_load} > {cpu_count * self.general_thresholds['max_load_average']}")
            validation['passed'] = False
        
        return validation

    def test_memory_bandwidth_comprehensive(self):
        """Comprehensive memory bandwidth testing"""
        self.log("Testing comprehensive memory bandwidth using stress-ng...")
        
        try:
            system_info = self.results.get('system_info', {})
            memory_info = psutil.virtual_memory()
            
            # Calculate theoretical bandwidth
            memory_channels = system_info.get('memory_channels', 4)
            memory_speed_mhz = system_info.get('memory_speed_mhz', 3200)
            memory_type = system_info.get('memory_type', 'Unknown')
            total_gb = memory_info.total / (1024**3)
            
            theoretical_bandwidth_gbps = (memory_speed_mhz * 2 * 8 * memory_channels) / 1000
            
            if self.verbose:
                print(f"\nMemory configuration for bandwidth testing:")
                print(f"  Detected Type: {memory_type}")
                print(f"  Detected Speed: {memory_speed_mhz} MHz")
                print(f"  Estimated Channels: {memory_channels}")
                print(f"  Total Memory: {total_gb:.1f} GB")
                print(f"  Theoretical Bandwidth: {theoretical_bandwidth_gbps:.1f} GB/s")
                print(f"    Formula: {memory_speed_mhz} MHz × 2 (DDR) × 8 bytes × {memory_channels} channels ÷ 1000")

            # Run bandwidth tests
            bandwidth_results = {
                'theoretical_bandwidth_gbps': theoretical_bandwidth_gbps,
                'memory_config': {'channels': memory_channels, 'speed_mhz': memory_speed_mhz, 'type': memory_type, 'total_gb': total_gb},
                'tests': {}
            }
            
            # Stream test (primary)
            self.log("Running stress-ng stream test...")
            stream_result = self.run_stress_ng_stream_test()
            bandwidth_results['tests']['stress_ng_stream'] = stream_result
            
            # VM test (secondary)
            self.log("Running stress-ng VM test...")
            vm_result = self.run_stress_ng_memory_bandwidth_test()
            bandwidth_results['tests']['stress_ng_vm'] = vm_result
            
            # Extract best bandwidth
            best_bandwidth_gbps = 0
            read_bandwidth_gbps = write_bandwidth_gbps = 0
            
            if stream_result.get('total_bandwidth_gbps', 0) > 0:
                best_bandwidth_gbps = stream_result['total_bandwidth_gbps']
                read_bandwidth_gbps = stream_result.get('read_bandwidth_gbps', 0)
                write_bandwidth_gbps = stream_result.get('write_bandwidth_gbps', 0)
            elif stream_result.get('bandwidth_gbps', 0) > 0:
                best_bandwidth_gbps = stream_result['bandwidth_gbps']
            
            # Calculate efficiency
            bandwidth_efficiency = (best_bandwidth_gbps / theoretical_bandwidth_gbps) * 100 if theoretical_bandwidth_gbps > 0 else 0
            
            # Get memory type thresholds
            mem_type_key = next((k for k in self.memory_thresholds.keys() if memory_type.startswith(k)), 'Unknown')
            thresholds = self.memory_thresholds[mem_type_key]
            min_bandwidth_threshold = thresholds['min_bandwidth_gbps']
            
            test_result = {
                'status': 'PASS' if best_bandwidth_gbps >= min_bandwidth_threshold else 'FAIL',
                'bandwidth_results': bandwidth_results,
                'best_measured_gbps': best_bandwidth_gbps,
                'theoretical_gbps': theoretical_bandwidth_gbps,
                'efficiency_percent': bandwidth_efficiency,
                'read_bandwidth_gbps': read_bandwidth_gbps,
                'write_bandwidth_gbps': write_bandwidth_gbps,
                'validation': self.validate_memory_performance(bandwidth_results, best_bandwidth_gbps, theoretical_bandwidth_gbps)
            }
            
            # Add warnings for poor performance
            warnings = []
            if bandwidth_efficiency < 20 and best_bandwidth_gbps > 0:
                warnings.extend([
                    f"Memory bandwidth may be underperforming: {best_bandwidth_gbps:.1f}GB/s vs {theoretical_bandwidth_gbps:.1f}GB/s theoretical ({bandwidth_efficiency:.1f}%)",
                    "Consider checking BIOS settings, memory configuration, or NUMA topology"
                ])
            elif best_bandwidth_gbps == 0:
                warnings.extend([
                    "Could not extract bandwidth measurements from stress-ng output",
                    "Memory stress test completed but bandwidth analysis limited"
                ])
            
            if memory_type == 'Unknown':
                warnings.extend([
                    "Could not detect memory type - using conservative thresholds",
                    "Run as root (sudo) for better memory detection via dmidecode"
                ])
            
            if warnings:
                test_result['warnings'] = warnings
            
            return test_result
            
        except Exception as e:
            return {'status': 'ERROR', 'error': str(e)}

    def run_stress_ng_stream_test(self):
        """Run stress-ng stream test for bandwidth measurement"""
        try:
            cmd = ['stress-ng', '--stream', '1', '--timeout', f'{self.test_duration}s', '--metrics-brief']
            if self.verbose:
                cmd.append('--verbose')
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=self.test_duration + 30)
            metrics = self.parse_stress_ng_output(result.stdout, result.stderr)
            
            # Calculate bandwidth in GB/s
            read_gbps = metrics['metrics'].get('read_bandwidth_mbps', 0) / 1000
            write_gbps = metrics['metrics'].get('write_bandwidth_mbps', 0) / 1000
            total_gbps = read_gbps + write_gbps
            
            return {
                'status': 'PASS' if result.returncode == 0 else 'FAIL',
                'bogo_ops_per_sec': metrics['metrics'].get('bogo_ops_per_sec', 0),
                'read_bandwidth_gbps': read_gbps,
                'write_bandwidth_gbps': write_gbps,
                'total_bandwidth_gbps': total_gbps,
                'bandwidth_gbps': total_gbps,
                'throughput_mbps': metrics['metrics'].get('total_bandwidth_mbps', 0),
                'errors': metrics['errors'],
                'warnings': metrics['warnings']
            }
        except Exception as e:
            return {'status': 'ERROR', 'error': str(e)}

    def run_stress_ng_memory_bandwidth_test(self):
        """Run stress-ng VM test with reasonable memory allocation"""
        try:
            available_gb = psutil.virtual_memory().available / (1024**3)
            total_gb = psutil.virtual_memory().total / (1024**3)
            test_memory_gb = max(1, int(min(available_gb * 0.8, total_gb * 0.5, 64)))
            
            if self.verbose:
                print(f"  VM test using {test_memory_gb}GB (available: {available_gb:.1f}GB, total: {total_gb:.1f}GB)")
            
            cmd = ['stress-ng', '--vm', '2', '--vm-bytes', f'{test_memory_gb}G', '--vm-method', 'all',
                   '--timeout', f'{self.test_duration}s', '--metrics-brief', '--verify']
            if self.verbose:
                cmd.append('--verbose')
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=self.test_duration + 30)
            metrics = self.parse_stress_ng_output(result.stdout, result.stderr)
            
            # Estimate bandwidth from bogo ops
            bogo_ops = metrics['metrics'].get('bogo_ops_per_sec', 0)
            estimated_bandwidth_gbps = (bogo_ops * 0.001) if bogo_ops > 0 else 0
            
            return {
                'status': 'PASS' if result.returncode == 0 else 'FAIL',
                'test_memory_gb': test_memory_gb,
                'bogo_ops_per_sec': bogo_ops,
                'throughput_mbps': metrics['metrics'].get('throughput_mbps', 0),
                'bandwidth_gbps': estimated_bandwidth_gbps,
                'errors': metrics['errors'],
                'warnings': metrics['warnings']
            }
        except Exception as e:
            return {'status': 'ERROR', 'error': str(e)}

    def validate_memory_performance(self, bandwidth_results, best_bandwidth_gbps, theoretical_bandwidth_gbps):
        """Validate memory performance with type-specific thresholds"""
        validation = {'passed': True, 'warnings': [], 'failures': [], 'recommendations': []}
        
        efficiency = (best_bandwidth_gbps / theoretical_bandwidth_gbps * 100) if theoretical_bandwidth_gbps > 0 else 0
        memory_type = bandwidth_results.get('memory_config', {}).get('type', 'Unknown')
        memory_speed = bandwidth_results.get('memory_config', {}).get('speed_mhz', 0)
        memory_channels = bandwidth_results.get('memory_config', {}).get('channels', 0)
        
        # Get type-specific thresholds
        mem_type_key = next((k for k in self.memory_thresholds.keys() if memory_type.startswith(k)), 'Unknown')
        thresholds = self.memory_thresholds[mem_type_key]
        
        # Adjust thresholds for dual-channel DDR4
        if memory_type.startswith('DDR4') and memory_channels == 2:
            thresholds = {**thresholds, 'efficiency_critical': 20, 'efficiency_warning': 40}
        
        profile_recommendations = {
            'DDR5': "Enable XMP/EXPO profiles in BIOS for DDR5",
            'DDR4': "Enable XMP profiles in BIOS for DDR4", 
            'DDR3': "Check memory speed settings in BIOS",
            'Unknown': "Check BIOS memory settings"
        }
        
        profile_recommendation = profile_recommendations.get(mem_type_key, profile_recommendations['Unknown'])
        
        # Validate efficiency
        if efficiency < thresholds['efficiency_critical']:
            validation['failures'].append(f"Memory bandwidth critically low: {best_bandwidth_gbps:.1f}GB/s ({efficiency:.1f}% of theoretical {theoretical_bandwidth_gbps:.1f}GB/s)")
            validation['passed'] = False
            validation['recommendations'].extend([
                profile_recommendation,
                "Verify memory is running at rated speed",
                "Ensure memory is populated across all available channels"
            ])
        elif efficiency < thresholds['efficiency_warning']:
            # Don't warn for good dual-channel DDR4 performance
            if not (memory_type.startswith('DDR4') and memory_channels == 2 and efficiency > 25):
                validation['warnings'].append(f"Memory bandwidth below expectations: {best_bandwidth_gbps:.1f}GB/s ({efficiency:.1f}% of theoretical)")
                validation['recommendations'].append("Memory performance may be suboptimal - check system configuration")
        
        # Speed validation
        expected_min_speeds = {'DDR5': 4800, 'DDR4': 2400, 'DDR3': 1333}
        expected_min_speed = expected_min_speeds.get(mem_type_key, 0)
        
        if memory_type != 'Unknown' and memory_speed > 0 and memory_speed < expected_min_speed:
            validation['warnings'].append(f"{memory_type} running at {memory_speed}MHz - below typical speeds")
            validation['recommendations'].append(profile_recommendation)
        
        # Add positive feedback for good performance
        if efficiency > 25 and memory_type.startswith('DDR4') and memory_channels == 2:
            validation['recommendations'].append(f"Dual-channel DDR4 performance is good at {best_bandwidth_gbps:.1f}GB/s ({efficiency:.1f}% efficiency)")
        elif efficiency > 15 and memory_type.startswith('DDR5') and memory_channels >= 6:
            validation['recommendations'].append(f"High-end DDR5 performance is acceptable at {best_bandwidth_gbps:.1f}GB/s ({efficiency:.1f}% efficiency)")
        
        return validation

    def analyze_system_configuration(self):
        """Analyze system configuration for optimal performance"""
        self.log("Analyzing system configuration...")
        
        try:
            system_info = self.results.get('system_info', {})
            memory_channels = system_info.get('memory_channels', 0)
            memory_speed_mhz = system_info.get('memory_speed_mhz', 0)
            memory_type = system_info.get('memory_type', 'Unknown')
            total_gb = system_info.get('memory_total_gb', 0)
            
            analysis = {
                'memory_config': {'channels': memory_channels, 'speed_mhz': memory_speed_mhz, 'type': memory_type, 'total_gb': total_gb},
                'recommendations': [],
                'potential_issues': []
            }
            
            # Memory type-specific analysis
            type_configs = {
                'DDR5': {'min_speed': 4800, 'profile': 'XMP/EXPO', 'min_capacity': 32},
                'DDR4': {'min_speed': 2400, 'profile': 'XMP', 'min_capacity': 16},
                'DDR3': {'min_speed': 1333, 'profile': 'memory speed', 'min_capacity': 8}
            }
            
            for mem_type, config in type_configs.items():
                if memory_type.startswith(mem_type):
                    if memory_speed_mhz < config['min_speed']:
                        analysis['potential_issues'].append(f"{memory_type} running at {memory_speed_mhz}MHz - may not be at rated speed")
                        analysis['recommendations'].append(f"Check BIOS settings for {config['profile']} memory profiles")
                    
                    if total_gb < config['min_capacity']:
                        analysis['potential_issues'].append(f"Low memory capacity ({total_gb}GB) may limit AI workload performance")
                        analysis['recommendations'].append(f"Consider increasing memory capacity for AI workloads")
                    break
            
            # Channel configuration analysis
            if memory_channels < 4 and total_gb > 64:
                analysis['potential_issues'].append(f"High memory capacity ({total_gb}GB) with low channel count ({memory_channels}) may limit bandwidth")
                analysis['recommendations'].append("Verify memory is populated across all available channels")
            
            # Tenstorrent card compatibility
            tt_cards = system_info.get('tenstorrent_cards', [])
            if tt_cards:
                analysis['recommendations'].append(f"Found {len(tt_cards)} Tenstorrent card(s) - ensure sufficient system resources")
            
            return {
                'status': 'PASS',
                'analysis': analysis,
                'recommendations': analysis['recommendations'],
                'potential_issues': analysis['potential_issues']
            }
            
        except Exception as e:
            return {'status': 'ERROR', 'error': str(e)}

    def test_cpu_stress(self):
        """Test CPU under heavy load"""
        cpu_count = psutil.cpu_count()
        return self.run_stress_test('CPU Stress Test', ['--cpu', str(cpu_count), '--cpu-method', 'all'])

    def test_memory_stress(self):
        """Basic memory stress test"""
        available_gb = psutil.virtual_memory().available / (1024**3)
        test_memory = max(1, int(available_gb * 0.8))
        return self.run_stress_test('Memory Stress Test', ['--vm', '2', '--vm-bytes', f'{test_memory}G', '--vm-method', 'all'])

    def test_combined_stress(self):
        """Test CPU and memory together"""
        cpu_count = psutil.cpu_count()
        available_gb = psutil.virtual_memory().available / (1024**3)
        test_memory = max(1, int(available_gb * 0.6))
        
        return self.run_stress_test('Combined System Stress Test', [
            '--cpu', str(max(1, cpu_count // 2)),
            '--vm', '1', '--vm-bytes', f'{test_memory}G',
            '--io', '2'
        ])

    def cleanup_after_tests(self):
        """Cleanup lingering processes"""
        self.log("Performing post-test cleanup...")
        
        try:
            time.sleep(2)
            for proc in psutil.process_iter(['pid', 'name']):
                try:
                    if proc.info and 'stress-ng' in str(proc.info.get('name', '')):
                        self.log(f"Terminating lingering stress-ng process: {proc.info['pid']}")
                        proc.terminate()
                        proc.wait(timeout=5)
                except:
                    pass
        except Exception as e:
            self.log(f"Cleanup warning: {e}")
        
        # Force garbage collection
        try:
            import gc
            gc.collect()
        except:
            pass

    def run_all_tests(self):
        """Run complete system validation suite"""
        print("tt-sysval: Tenstorrent System Validation Suite")
        print("=" * 60)
        
        if not self.check_dependencies():
            return False
        
        # Gather system info
        system_info = self.gather_system_info()
        
        # Display system summary
        print(f"System: {system_info.get('cpu_details', {}).get('model_name', 'Unknown CPU')}")
        print(f"Cores: {system_info['cpu_count']} logical ({system_info['cpu_physical_cores']} physical)")
        print(f"Memory: {system_info['memory_total_gb']} GB")
        
        if system_info.get('memory_type'):
            mem_config = f"({system_info['memory_type']}"
            if system_info.get('memory_speed_mhz'):
                mem_config += f"-{system_info['memory_speed_mhz']}"
            if system_info.get('memory_channels'):
                mem_config += f", {system_info['memory_channels']} channels"
            mem_config += ")"
            print(f"Memory Config: {mem_config}")
        
        if system_info.get('tenstorrent_cards'):
            print(f"Tenstorrent Cards: {len(system_info['tenstorrent_cards'])} detected")
        
        print(f"\nRunning tests with {self.test_duration}s duration each...")
        print()
        
        # Test suite
        tests = [
            ('System Configuration Analysis', self.analyze_system_configuration),
            ('Comprehensive Memory Bandwidth', self.test_memory_bandwidth_comprehensive),
            ('CPU Stress', self.test_cpu_stress),
            ('Memory Stress', self.test_memory_stress),
            ('Combined Stress', self.test_combined_stress)
        ]
        
        all_passed = True
        
        try:
            for test_name, test_func in tests:
                print(f"Running {test_name}...")
                result = test_func()
                status = result.get('status', 'UNKNOWN')
                print(f"  Status: {status}")
                
                if status == 'PASS':
                    self.print_test_metrics(result)
                else:
                    all_passed = False
                    if result.get('error'):
                        print(f"  Error: {result['error']}")
                    
                    # Show failures and warnings
                    validation = result.get('validation', {})
                    for failure in validation.get('failures', []):
                        print(f"    - {failure}")
                    for warning in result.get('warnings', []):
                        print(f"    - {warning}")
                
                print()
                self.results['tests'][test_name] = result
        
        finally:
            self.cleanup_after_tests()
        
        return all_passed

    def print_test_metrics(self, result):
        """Print test metrics in a consistent format"""
        validation = result.get('validation', {})
        if validation.get('warnings'):
            print(f"  Warnings: {len(validation['warnings'])}")
            for warning in validation['warnings']:
                print(f"    - {warning}")
        
        # System metrics
        metrics = result.get('system_metrics', {})
        if 'cpu_usage' in metrics:
            print(f"  CPU Usage: {metrics['cpu_usage']['avg']}% avg, {metrics['cpu_usage']['max']}% max")
        if 'memory_usage' in metrics:
            print(f"  Memory Usage: {metrics['memory_usage']['avg']}% avg, {metrics['memory_usage']['max']}% max")
        if 'temperature' in metrics:
            print(f"  Max Temperature: {metrics['temperature']['max']}°C")
        
        # Memory bandwidth results
        if 'best_measured_gbps' in result:
            print(f"  Memory Bandwidth: {result['best_measured_gbps']:.1f}GB/s ({result.get('efficiency_percent', 0):.1f}% of theoretical)")
            if result.get('theoretical_gbps', 0) > 0:
                print(f"  Theoretical Max: {result['theoretical_gbps']:.1f}GB/s")
            if result.get('read_bandwidth_gbps', 0) > 0:
                print(f"  Read: {result['read_bandwidth_gbps']:.1f}GB/s, Write: {result.get('write_bandwidth_gbps', 0):.1f}GB/s")

    def generate_validation_summary(self):
        """Generate overall system validation summary"""
        summary = {'overall_status': 'UNKNOWN', 'deployment_ready': False, 'critical_issues': [], 'warnings': [], 'recommendations': []}
        
        all_passed = True
        has_warnings = False
        
        for test_name, result in self.results['tests'].items():
            status = result.get('status')
            validation = result.get('validation', {})
            
            if status != 'PASS':
                all_passed = False
                summary['critical_issues'].append(f"{test_name}: {status}")
            
            # Collect issues and recommendations
            for failure in validation.get('failures', []):
                summary['critical_issues'].append(f"{test_name}: {failure}")
            for warning in validation.get('warnings', []):
                has_warnings = True
                summary['warnings'].append(f"{test_name}: {warning}")
            for warning in result.get('warnings', []):
                has_warnings = True
                summary['warnings'].append(f"{test_name}: {warning}")
            for rec in validation.get('recommendations', []):
                summary['recommendations'].append(rec)
        
        # Determine overall status
        if all_passed and not summary['critical_issues']:
            if has_warnings:
                summary['overall_status'] = 'PASS_WITH_WARNINGS'
                summary['deployment_ready'] = True
                summary['recommendations'].append("System is suitable for AI accelerator deployment, but monitor the warnings")
            else:
                summary['overall_status'] = 'PASS'
                summary['deployment_ready'] = True
                summary['recommendations'].append("System is ready for Tenstorrent AI accelerator deployment")
        else:
            summary['overall_status'] = 'FAIL'
            summary['recommendations'].append("Address critical issues before AI accelerator deployment")
            
            # Add specific recommendations based on failure types
            if any('temperature' in issue.lower() for issue in summary['critical_issues']):
                summary['recommendations'].append("Improve system cooling before adding thermal load")
            if any(keyword in issue.lower() for keyword in ['memory', 'bandwidth'] for issue in summary['critical_issues'] + summary['warnings']):
                summary['recommendations'].extend([
                    "Check memory configuration and BIOS settings",
                    "Verify XMP/EXPO profiles are enabled",
                    "Ensure memory is populated across all channels"
                ])
        
        summary['recommendations'] = list(dict.fromkeys(summary['recommendations']))  # Remove duplicates
        self.results['validation_summary'] = summary
        return summary

    def print_summary_report(self):
        """Print final summary report"""
        summary = self.generate_validation_summary()
        
        print("\n" + "=" * 60)
        print("TENSTORRENT SYSTEM VALIDATION SUMMARY")
        print("=" * 60)
        
        print(f"Overall Status: {summary['overall_status']}")
        print(f"Deployment Ready: {'YES' if summary['deployment_ready'] else 'NO'}")
        
        # Memory performance analysis
        bandwidth_test = self.results.get('tests', {}).get('Comprehensive Memory Bandwidth', {})
        if bandwidth_test.get('best_measured_gbps', 0) > 0:
            memory_type = bandwidth_test.get('bandwidth_results', {}).get('memory_config', {}).get('type', '')
            measured_gbps = bandwidth_test.get('best_measured_gbps', 0)
            theoretical_gbps = bandwidth_test.get('theoretical_gbps', 0)
            efficiency = bandwidth_test.get('efficiency_percent', 0)
            
            print(f"\nMemory Performance Analysis:")
            print(f"  Measured Bandwidth: {measured_gbps:.1f} GB/s")
            print(f"  Theoretical Maximum: {theoretical_gbps:.1f} GB/s")
            print(f"  Efficiency: {efficiency:.1f}%")
            
            if memory_type.startswith('DDR5') and efficiency < 25:
                print(f"  Note: DDR5 efficiency of {efficiency:.1f}% is common due to higher latency")
                print(f"        Actual bandwidth of {measured_gbps:.1f}GB/s may be reasonable for your system")
        
        # Print issues and recommendations
        for section, items, icon in [
            ('Critical Issues', summary['critical_issues'], '❌'),
            ('Warnings', summary['warnings'], '⚠️'),
            ('Recommendations', summary['recommendations'], '💡')
        ]:
            if items:
                print(f"\n{section} ({len(items)}):")
                for item in items:
                    print(f"  {icon} {item}")
        
        return summary['deployment_ready']

def main():
    parser = argparse.ArgumentParser(
        description='tt-sysval: Tenstorrent System Validation Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 tt-sysval.py                      # Quick 30-second tests
  python3 tt-sysval.py -d 60 -v             # 60-second tests with verbose output
  python3 tt-sysval.py -d 300 -o report.json # 5-minute stress test with report
        """
    )
    
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output during testing')
    parser.add_argument('-d', '--duration', type=int, default=30, help='Duration in seconds for each stress test (default: 30)')
    parser.add_argument('-o', '--output', help='Output file for detailed JSON report')
    
    args = parser.parse_args()
    
    if args.duration < 10:
        print("Warning: Duration less than 10 seconds may not provide reliable results")
    
    validator = None
    exit_code = 0
    
    try:
        validator = TenstorrentSystemValidator(verbose=args.verbose, duration=args.duration)
        success = validator.run_all_tests()
        deployment_ready = validator.print_summary_report()
        
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(validator.results, f, indent=2)
            print(f"\nDetailed report saved to: {args.output}")
        
        # Determine exit code
        exit_code = 0 if deployment_ready else (2 if not success else 1)
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        exit_code = 130
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        exit_code = 2
    finally:
        # Comprehensive cleanup
        if validator:
            validator.cleanup_after_tests()
        
        try:
            import gc, threading
            # Clean up threads
            for thread in threading.enumerate():
                if thread != threading.current_thread() and thread.is_alive():
                    thread.daemon = True
            gc.collect()
        except:
            pass
        
        # Use os._exit to avoid Python cleanup issues
        try:
            import os
            os._exit(exit_code)
        except:
            sys.exit(exit_code)

if __name__ == '__main__':
    main()
