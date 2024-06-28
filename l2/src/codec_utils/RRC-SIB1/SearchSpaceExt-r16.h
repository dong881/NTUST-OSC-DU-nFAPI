/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SearchSpaceExt_r16_H_
#define	_SearchSpaceExt_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ControlResourceSetId-r16.h"
#include <BIT_STRING.h>
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>
#include <NativeInteger.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel1_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel1_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel1_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel1_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel2_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel2_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel2_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel2_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel4_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel4_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel4_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel4_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel8_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel8_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel8_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel8_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel16_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel16_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel16_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16__aggregationLevel16_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel1_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel1_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel1_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel1_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel2_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel2_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel2_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel2_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel4_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel4_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel4_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel4_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel8_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel8_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel8_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel8_r16;
typedef enum SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel16_r16 {
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel16_r16_n1	= 0,
	SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel16_r16_n2	= 1
} e_SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16__aggregationLevel16_r16;

/* SearchSpaceExt-r16 */
typedef struct SearchSpaceExt_r16 {
	ControlResourceSetId_r16_t	*controlResourceSetId_r16;	/* OPTIONAL */
	struct SearchSpaceExt_r16__searchSpaceType_r16 {
		struct SearchSpaceExt_r16__searchSpaceType_r16__common_r16 {
			struct SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16 {
				struct SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_4_r16__nrofCandidates_CI_r16 {
					long	*aggregationLevel1_r16;	/* OPTIONAL */
					long	*aggregationLevel2_r16;	/* OPTIONAL */
					long	*aggregationLevel4_r16;	/* OPTIONAL */
					long	*aggregationLevel8_r16;	/* OPTIONAL */
					long	*aggregationLevel16_r16;	/* OPTIONAL */
					
					/* Context for parsing across buffer boundaries */
					asn_struct_ctx_t _asn_ctx;
				} nrofCandidates_CI_r16;
				/*
				 * This type is extensible,
				 * possible extensions are below.
				 */
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *dci_Format2_4_r16;
			struct SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16 {
				struct SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_5_r16__nrofCandidates_IAB_r16 {
					long	*aggregationLevel1_r16;	/* OPTIONAL */
					long	*aggregationLevel2_r16;	/* OPTIONAL */
					long	*aggregationLevel4_r16;	/* OPTIONAL */
					long	*aggregationLevel8_r16;	/* OPTIONAL */
					long	*aggregationLevel16_r16;	/* OPTIONAL */
					
					/* Context for parsing across buffer boundaries */
					asn_struct_ctx_t _asn_ctx;
				} nrofCandidates_IAB_r16;
				/*
				 * This type is extensible,
				 * possible extensions are below.
				 */
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *dci_Format2_5_r16;
			struct SearchSpaceExt_r16__searchSpaceType_r16__common_r16__dci_Format2_6_r16 {
				/*
				 * This type is extensible,
				 * possible extensions are below.
				 */
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *dci_Format2_6_r16;
			/*
			 * This type is extensible,
			 * possible extensions are below.
			 */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} common_r16;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *searchSpaceType_r16;
	struct SearchSpaceExt_r16__searchSpaceGroupIdList_r16 {
		A_SEQUENCE_OF(long) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *searchSpaceGroupIdList_r16;
	BIT_STRING_t	*freqMonitorLocations_r16;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SearchSpaceExt_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel1_r16_7;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel2_r16_10;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel4_r16_13;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel8_r16_16;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel16_r16_19;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel1_r16_25;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel2_r16_28;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel4_r16_31;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel8_r16_34;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_aggregationLevel16_r16_37;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SearchSpaceExt_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SearchSpaceExt_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SearchSpaceExt_r16_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _SearchSpaceExt_r16_H_ */
#include <asn_internal.h>
