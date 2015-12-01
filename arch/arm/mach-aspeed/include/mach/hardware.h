/*
 *  hardware.h
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <mach/platform.h>

#define IO_ADDRESS(x)  (x - 0x1e600000 + 0xF8000000) 

//PCIE
#ifdef CONFIG_PCIE_AST
#define pcibios_assign_all_busses()	1
#endif

#endif	/* __ASM_ARCH_HARDWARE_H END */

