#ifndef __HVAC_COMPILER_H__
#define __HVAC_COMPILER_H__

#define __stringify__(s...) #s
#define __stringify(s...) __stringify__(s)

#if defined(__GNUC__)
#define __ccdiag(s) _Pragma(__stringify(GCC diagnostic s))
#elif defined(__clang__)
#define __ccdiag(s) _Pragma(__stringify(clang diagnostic s))
#else
#define __ccdiag(s)
#endif

#define __ccignore(warn, instr) ({						\
	__ccdiag(push)								\
	__ccdiag(ignored __stringify(-W ## warn))				\
	instr;									\
	__ccdiag(pop)								\
})

#endif
