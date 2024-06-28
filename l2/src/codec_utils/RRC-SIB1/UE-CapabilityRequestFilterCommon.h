/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_UE_CapabilityRequestFilterCommon_H_
#define	_UE_CapabilityRequestFilterCommon_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_SEQUENCE.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum UE_CapabilityRequestFilterCommon__mrdc_Request__omitEN_DC {
	UE_CapabilityRequestFilterCommon__mrdc_Request__omitEN_DC_true	= 0
} e_UE_CapabilityRequestFilterCommon__mrdc_Request__omitEN_DC;
typedef enum UE_CapabilityRequestFilterCommon__mrdc_Request__includeNR_DC {
	UE_CapabilityRequestFilterCommon__mrdc_Request__includeNR_DC_true	= 0
} e_UE_CapabilityRequestFilterCommon__mrdc_Request__includeNR_DC;
typedef enum UE_CapabilityRequestFilterCommon__mrdc_Request__includeNE_DC {
	UE_CapabilityRequestFilterCommon__mrdc_Request__includeNE_DC_true	= 0
} e_UE_CapabilityRequestFilterCommon__mrdc_Request__includeNE_DC;
typedef enum UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type1_SinglePanel_r16 {
	UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type1_SinglePanel_r16_true	= 0
} e_UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type1_SinglePanel_r16;
typedef enum UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type1_MultiPanel_r16 {
	UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type1_MultiPanel_r16_true	= 0
} e_UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type1_MultiPanel_r16;
typedef enum UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type2_r16 {
	UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type2_r16_true	= 0
} e_UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type2_r16;
typedef enum UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type2_PortSelection_r16 {
	UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type2_PortSelection_r16_true	= 0
} e_UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16__type2_PortSelection_r16;
typedef enum UE_CapabilityRequestFilterCommon__ext1__uplinkTxSwitchRequest_r16 {
	UE_CapabilityRequestFilterCommon__ext1__uplinkTxSwitchRequest_r16_true	= 0
} e_UE_CapabilityRequestFilterCommon__ext1__uplinkTxSwitchRequest_r16;
typedef enum UE_CapabilityRequestFilterCommon__ext3__fallbackGroupFiveRequest_r17 {
	UE_CapabilityRequestFilterCommon__ext3__fallbackGroupFiveRequest_r17_true	= 0
} e_UE_CapabilityRequestFilterCommon__ext3__fallbackGroupFiveRequest_r17;

/* Forward declarations */
struct CellGrouping_r16;

/* UE-CapabilityRequestFilterCommon */
typedef struct UE_CapabilityRequestFilterCommon {
	struct UE_CapabilityRequestFilterCommon__mrdc_Request {
		long	*omitEN_DC;	/* OPTIONAL */
		long	*includeNR_DC;	/* OPTIONAL */
		long	*includeNE_DC;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *mrdc_Request;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct UE_CapabilityRequestFilterCommon__ext1 {
		struct UE_CapabilityRequestFilterCommon__ext1__codebookTypeRequest_r16 {
			long	*type1_SinglePanel_r16;	/* OPTIONAL */
			long	*type1_MultiPanel_r16;	/* OPTIONAL */
			long	*type2_r16;	/* OPTIONAL */
			long	*type2_PortSelection_r16;	/* OPTIONAL */
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *codebookTypeRequest_r16;
		long	*uplinkTxSwitchRequest_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct UE_CapabilityRequestFilterCommon__ext2 {
		struct UE_CapabilityRequestFilterCommon__ext2__requestedCellGrouping_r16 {
			A_SEQUENCE_OF(struct CellGrouping_r16) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *requestedCellGrouping_r16;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct UE_CapabilityRequestFilterCommon__ext3 {
		long	*fallbackGroupFiveRequest_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} UE_CapabilityRequestFilterCommon_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_omitEN_DC_3;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_includeNR_DC_5;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_includeNE_DC_7;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_type1_SinglePanel_r16_12;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_type1_MultiPanel_r16_14;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_type2_r16_16;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_type2_PortSelection_r16_18;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_uplinkTxSwitchRequest_r16_20;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_fallbackGroupFiveRequest_r17_26;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_UE_CapabilityRequestFilterCommon;
extern asn_SEQUENCE_specifics_t asn_SPC_UE_CapabilityRequestFilterCommon_specs_1;
extern asn_TYPE_member_t asn_MBR_UE_CapabilityRequestFilterCommon_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "CellGrouping-r16.h"

#endif	/* _UE_CapabilityRequestFilterCommon_H_ */
#include <asn_internal.h>
