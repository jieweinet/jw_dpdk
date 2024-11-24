# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2017 Intel Corporation


drivers = [
		'i350',
]
std_deps = ['ethdev', 'kvargs'] # 'ethdev' also pulls in mbuf, net, eal etc
std_deps += ['bus_pci']         # very many PMDs depend on PCI, so make std
std_deps += ['bus_vdev']        # same with vdev bus
