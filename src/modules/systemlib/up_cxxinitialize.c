
/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

//#include <debug.h>

//#include <nuttx/arch.h>

//#include <arch/stm32/chip.h>
//#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Debug ****************************************************************************/
/* Non-standard debug that may be enabled just for testing the static constructors */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_CXX
#endif

#ifdef CONFIG_DEBUG_CXX
#  define cxxdbg              dbg
#  define cxxlldbg            lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define cxxvdbg           vdbg
#    define cxxllvdbg         llvdbg
#  else
#    define cxxvdbg(x...)
#    define cxxllvdbg(x...)
#  endif
#else
#  define cxxdbg(x...)
#  define cxxlldbg(x...)
#  define cxxvdbg(x...)
#  define cxxllvdbg(x...)
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/
/* This type defines one entry in initialization array */

typedef void (*initializer_t)(void);

/************************************************************************************
 * External references
 ************************************************************************************/
/* _sinit and _einit are symbols exported by the linker script that mark the
 * beginning and the end of the C++ initialization section.
 * _sinit 和 _einit
 */

extern initializer_t _sinit;
extern initializer_t _einit;

/* _stext and _etext are symbols exported by the linker script that mark the
 * beginning and the end of text.
 */

extern uint32_t _stext;
extern uint32_t _etext;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: up_cxxinitialize
 *
 * Description:
 *   If C++ and C++ static constructors are supported, then this function
 *   must be provided by board-specific logic in order to perform
 *   initialization of the static C++ class instances.
 *
 *   This function should then be called in the application-specific
 *   user_start logic in order to perform the C++ initialization.  NOTE
 *   that no component of the core NuttX RTOS logic is involved; This
 *   function definition only provides the 'contract' between application
 *   specific C++ code and platform-specific toolchain support
 *
 ***************************************************************************/

__EXPORT void up_cxxinitialize(void);
void up_cxxinitialize(void)
{
	initializer_t *initp;

	cxxdbg("_sinit: %p _einit: %p _stext: %p _etext: %p\n",
	       &_sinit, &_einit, &_stext, &_etext);

	/* Visit each entry in the initialzation table */

	for (initp = &_sinit; initp != &_einit; initp++) {
		initializer_t initializer = *initp;
		cxxdbg("initp: %p initializer: %p\n", initp, initializer);

		/* Make sure that the address is non-NULL and lies in the text region
		 * defined by the linker script.  Some toolchains may put NULL values
		 * or counts in the initialization table
		 */

		if ((void *)initializer > (void *)&_stext && (void *)initializer < (void *)&_etext) {
			cxxdbg("Calling %p\n", initializer);
			initializer();
		}
	}
}
