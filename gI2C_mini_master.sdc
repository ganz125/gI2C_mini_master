#  ------------------------------------------------------------------------------
#
#  gI2C_mini_master.sdc -- sample SDC file for gI2C_mini_master project
#
#  Copyright (C) 2020 Michael Gansler
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#  ------------------------------------------------------------------------------
create_clock -name "CLOCK_50" -period 20.0ns [get_ports {CLOCK_50}]
derive_pll_clocks
derive_clock_uncertainty
