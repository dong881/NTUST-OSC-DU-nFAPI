/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_PDSCH_HARQ_ACK_EnhType3_r17_H_
#define	_PDSCH_HARQ_ACK_EnhType3_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include "PDSCH-HARQ-ACK-EnhType3Index-r17.h"
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <BIT_STRING.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17_PR {
	PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17_PR_NOTHING,	/* No components present */
	PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17_PR_perCC,
	PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17_PR_perHARQ
} PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17_PR;
typedef enum PDSCH_HARQ_ACK_EnhType3_r17__pdsch_HARQ_ACK_EnhType3NDI_r17 {
	PDSCH_HARQ_ACK_EnhType3_r17__pdsch_HARQ_ACK_EnhType3NDI_r17_true	= 0
} e_PDSCH_HARQ_ACK_EnhType3_r17__pdsch_HARQ_ACK_EnhType3NDI_r17;
typedef enum PDSCH_HARQ_ACK_EnhType3_r17__pdsch_HARQ_ACK_EnhType3CBG_r17 {
	PDSCH_HARQ_ACK_EnhType3_r17__pdsch_HARQ_ACK_EnhType3CBG_r17_true	= 0
} e_PDSCH_HARQ_ACK_EnhType3_r17__pdsch_HARQ_ACK_EnhType3CBG_r17;

/* PDSCH-HARQ-ACK-EnhType3-r17 */
typedef struct PDSCH_HARQ_ACK_EnhType3_r17 {
	PDSCH_HARQ_ACK_EnhType3Index_r17_t	 pdsch_HARQ_ACK_EnhType3Index_r17;
	struct PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17 {
		PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17_PR present;
		union PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17_u {
			struct PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17__perCC {
				A_SEQUENCE_OF(long) list;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *perCC;
			struct PDSCH_HARQ_ACK_EnhType3_r17__applicable_r17__perHARQ {
				A_SEQUENCE_OF(BIT_STRING_t) list;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *perHARQ;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} applicable_r17;
	long	*pdsch_HARQ_ACK_EnhType3NDI_r17;	/* OPTIONAL */
	long	*pdsch_HARQ_ACK_EnhType3CBG_r17;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PDSCH_HARQ_ACK_EnhType3_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_pdsch_HARQ_ACK_EnhType3NDI_r17_8;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pdsch_HARQ_ACK_EnhType3CBG_r17_10;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PDSCH_HARQ_ACK_EnhType3_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_PDSCH_HARQ_ACK_EnhType3_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_PDSCH_HARQ_ACK_EnhType3_r17_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _PDSCH_HARQ_ACK_EnhType3_r17_H_ */
#include <asn_internal.h>
