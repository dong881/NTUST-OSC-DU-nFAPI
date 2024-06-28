/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PDCP_Parameters_H_
#define	_PDCP_Parameters_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <BOOLEAN.h>
#include <constr_SEQUENCE.h>
#include <NativeInteger.h>
#include "PLMN-Identity.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PDCP_Parameters__maxNumberROHC_ContextSessions {
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs2	= 0,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs4	= 1,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs8	= 2,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs12	= 3,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs16	= 4,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs24	= 5,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs32	= 6,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs48	= 7,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs64	= 8,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs128	= 9,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs256	= 10,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs512	= 11,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs1024	= 12,
	PDCP_Parameters__maxNumberROHC_ContextSessions_cs16384	= 13,
	PDCP_Parameters__maxNumberROHC_ContextSessions_spare2	= 14,
	PDCP_Parameters__maxNumberROHC_ContextSessions_spare1	= 15
} e_PDCP_Parameters__maxNumberROHC_ContextSessions;
typedef enum PDCP_Parameters__uplinkOnlyROHC_Profiles {
	PDCP_Parameters__uplinkOnlyROHC_Profiles_supported	= 0
} e_PDCP_Parameters__uplinkOnlyROHC_Profiles;
typedef enum PDCP_Parameters__continueROHC_Context {
	PDCP_Parameters__continueROHC_Context_supported	= 0
} e_PDCP_Parameters__continueROHC_Context;
typedef enum PDCP_Parameters__outOfOrderDelivery {
	PDCP_Parameters__outOfOrderDelivery_supported	= 0
} e_PDCP_Parameters__outOfOrderDelivery;
typedef enum PDCP_Parameters__shortSN {
	PDCP_Parameters__shortSN_supported	= 0
} e_PDCP_Parameters__shortSN;
typedef enum PDCP_Parameters__pdcp_DuplicationSRB {
	PDCP_Parameters__pdcp_DuplicationSRB_supported	= 0
} e_PDCP_Parameters__pdcp_DuplicationSRB;
typedef enum PDCP_Parameters__pdcp_DuplicationMCG_OrSCG_DRB {
	PDCP_Parameters__pdcp_DuplicationMCG_OrSCG_DRB_supported	= 0
} e_PDCP_Parameters__pdcp_DuplicationMCG_OrSCG_DRB;
typedef enum PDCP_Parameters__ext1__drb_IAB_r16 {
	PDCP_Parameters__ext1__drb_IAB_r16_supported	= 0
} e_PDCP_Parameters__ext1__drb_IAB_r16;
typedef enum PDCP_Parameters__ext1__non_DRB_IAB_r16 {
	PDCP_Parameters__ext1__non_DRB_IAB_r16_supported	= 0
} e_PDCP_Parameters__ext1__non_DRB_IAB_r16;
typedef enum PDCP_Parameters__ext1__extendedDiscardTimer_r16 {
	PDCP_Parameters__ext1__extendedDiscardTimer_r16_supported	= 0
} e_PDCP_Parameters__ext1__extendedDiscardTimer_r16;
typedef enum PDCP_Parameters__ext1__continueEHC_Context_r16 {
	PDCP_Parameters__ext1__continueEHC_Context_r16_supported	= 0
} e_PDCP_Parameters__ext1__continueEHC_Context_r16;
typedef enum PDCP_Parameters__ext1__ehc_r16 {
	PDCP_Parameters__ext1__ehc_r16_supported	= 0
} e_PDCP_Parameters__ext1__ehc_r16;
typedef enum PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16 {
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs2	= 0,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs4	= 1,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs8	= 2,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs16	= 3,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs32	= 4,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs64	= 5,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs128	= 6,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs256	= 7,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs512	= 8,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs1024	= 9,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs2048	= 10,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs4096	= 11,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs8192	= 12,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs16384	= 13,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs32768	= 14,
	PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16_cs65536	= 15
} e_PDCP_Parameters__ext1__maxNumberEHC_Contexts_r16;
typedef enum PDCP_Parameters__ext1__jointEHC_ROHC_Config_r16 {
	PDCP_Parameters__ext1__jointEHC_ROHC_Config_r16_supported	= 0
} e_PDCP_Parameters__ext1__jointEHC_ROHC_Config_r16;
typedef enum PDCP_Parameters__ext1__pdcp_DuplicationMoreThanTwoRLC_r16 {
	PDCP_Parameters__ext1__pdcp_DuplicationMoreThanTwoRLC_r16_supported	= 0
} e_PDCP_Parameters__ext1__pdcp_DuplicationMoreThanTwoRLC_r16;
typedef enum PDCP_Parameters__ext2__longSN_RedCap_r17 {
	PDCP_Parameters__ext2__longSN_RedCap_r17_supported	= 0
} e_PDCP_Parameters__ext2__longSN_RedCap_r17;
typedef enum PDCP_Parameters__ext2__udc_r17__standardDictionary_r17 {
	PDCP_Parameters__ext2__udc_r17__standardDictionary_r17_supported	= 0
} e_PDCP_Parameters__ext2__udc_r17__standardDictionary_r17;
typedef enum PDCP_Parameters__ext2__udc_r17__continueUDC_r17 {
	PDCP_Parameters__ext2__udc_r17__continueUDC_r17_supported	= 0
} e_PDCP_Parameters__ext2__udc_r17__continueUDC_r17;
typedef enum PDCP_Parameters__ext2__udc_r17__supportOfBufferSize_r17 {
	PDCP_Parameters__ext2__udc_r17__supportOfBufferSize_r17_kbyte4	= 0,
	PDCP_Parameters__ext2__udc_r17__supportOfBufferSize_r17_kbyte8	= 1
} e_PDCP_Parameters__ext2__udc_r17__supportOfBufferSize_r17;

/* PDCP-Parameters */
typedef struct PDCP_Parameters {
	struct PDCP_Parameters__supportedROHC_Profiles {
		BOOLEAN_t	 profile0x0000;
		BOOLEAN_t	 profile0x0001;
		BOOLEAN_t	 profile0x0002;
		BOOLEAN_t	 profile0x0003;
		BOOLEAN_t	 profile0x0004;
		BOOLEAN_t	 profile0x0006;
		BOOLEAN_t	 profile0x0101;
		BOOLEAN_t	 profile0x0102;
		BOOLEAN_t	 profile0x0103;
		BOOLEAN_t	 profile0x0104;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} supportedROHC_Profiles;
	long	 maxNumberROHC_ContextSessions;
	long	*uplinkOnlyROHC_Profiles;	/* OPTIONAL */
	long	*continueROHC_Context;	/* OPTIONAL */
	long	*outOfOrderDelivery;	/* OPTIONAL */
	long	*shortSN;	/* OPTIONAL */
	long	*pdcp_DuplicationSRB;	/* OPTIONAL */
	long	*pdcp_DuplicationMCG_OrSCG_DRB;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct PDCP_Parameters__ext1 {
		long	*drb_IAB_r16;	/* OPTIONAL */
		long	*non_DRB_IAB_r16;	/* OPTIONAL */
		long	*extendedDiscardTimer_r16;	/* OPTIONAL */
		long	*continueEHC_Context_r16;	/* OPTIONAL */
		long	*ehc_r16;	/* OPTIONAL */
		long	*maxNumberEHC_Contexts_r16;	/* OPTIONAL */
		long	*jointEHC_ROHC_Config_r16;	/* OPTIONAL */
		long	*pdcp_DuplicationMoreThanTwoRLC_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct PDCP_Parameters__ext2 {
		long	*longSN_RedCap_r17;	/* OPTIONAL */
		struct PDCP_Parameters__ext2__udc_r17 {
			long	*standardDictionary_r17;	/* OPTIONAL */
			struct PDCP_Parameters__ext2__udc_r17__operatorDictionary_r17 {
				long	 versionOfDictionary_r17;
				PLMN_Identity_t	 associatedPLMN_ID_r17;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *operatorDictionary_r17;
			long	*continueUDC_r17;	/* OPTIONAL */
			long	*supportOfBufferSize_r17;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *udc_r17;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PDCP_Parameters_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumberROHC_ContextSessions_13;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_uplinkOnlyROHC_Profiles_30;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_continueROHC_Context_32;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_outOfOrderDelivery_34;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_shortSN_36;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pdcp_DuplicationSRB_38;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pdcp_DuplicationMCG_OrSCG_DRB_40;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_drb_IAB_r16_44;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_non_DRB_IAB_r16_46;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_extendedDiscardTimer_r16_48;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_continueEHC_Context_r16_50;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_ehc_r16_52;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_maxNumberEHC_Contexts_r16_54;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_jointEHC_ROHC_Config_r16_71;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pdcp_DuplicationMoreThanTwoRLC_r16_73;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_longSN_RedCap_r17_76;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_standardDictionary_r17_79;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_continueUDC_r17_84;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_supportOfBufferSize_r17_86;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PDCP_Parameters;
extern asn_SEQUENCE_specifics_t asn_SPC_PDCP_Parameters_specs_1;
extern asn_TYPE_member_t asn_MBR_PDCP_Parameters_1[10];

#ifdef __cplusplus
}
#endif

#endif	/* _PDCP_Parameters_H_ */
#include <asn_internal.h>
