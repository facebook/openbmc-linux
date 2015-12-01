/*
 *  memory.h
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
#include <mach/platform.h>
#include <plat/aspeed.h>

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */
#if defined(AST_SOC_G3) || defined(AST_SOC_G4)
#define PHYS_OFFSET	UL(0x40000000)
#define BUS_OFFSET	UL(0x40000000)
#elif defined(AST_SOC_G5)
#define PHYS_OFFSET     UL(0x80000000)
#define BUS_OFFSET      UL(0x80000000)
#else
#define PHYS_OFFSET     UL(0x40000000)
#define BUS_OFFSET      UL(0x40000000)
#endif

#endif
