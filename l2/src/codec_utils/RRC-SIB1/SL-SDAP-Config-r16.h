/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_SDAP_Config_r16_H_
#define	_SL_SDAP_Config_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <BOOLEAN.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_SDAP_Config_r16__sl_SDAP_Header_r16 {
	SL_SDAP_Config_r16__sl_SDAP_Header_r16_present	= 0,
	SL_SDAP_Config_r16__sl_SDAP_Header_r16_absent	= 1
} e_SL_SDAP_Config_r16__sl_SDAP_Header_r16;
typedef enum SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16_PR {
	SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16_PR_NOTHING,	/* No components present */
	SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16_PR_sl_MappedQoS_FlowsList_r16,
	SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16_PR_sl_MappedQoS_FlowsListDedicated_r16
} SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16_PR;
typedef enum SL_SDAP_Config_r16__sl_CastType_r16 {
	SL_SDAP_Config_r16__sl_CastType_r16_broadcast	= 0,
	SL_SDAP_Config_r16__sl_CastType_r16_groupcast	= 1,
	SL_SDAP_Config_r16__sl_CastType_r16_unicast	= 2,
	SL_SDAP_Config_r16__sl_CastType_r16_spare1	= 3
} e_SL_SDAP_Config_r16__sl_CastType_r16;

/* Forward declarations */
struct SL_MappedQoS_FlowsListDedicated_r16;
struct SL_QoS_Profile_r16;

/* SL-SDAP-Config-r16 */
typedef struct SL_SDAP_Config_r16 {
	long	 sl_SDAP_Header_r16;
	BOOLEAN_t	 sl_DefaultRB_r16;
	struct SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16 {
		SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16_PR present;
		union SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16_u {
			struct SL_SDAP_Config_r16__sl_MappedQoS_Flows_r16__sl_MappedQoS_FlowsList_r16 {
				A_SEQUENCE_OF(struct SL_QoS_Profile_r16) list;
				
				/* Context for parsing across buffer boundaries */
				asn_struct_ctx_t _asn_ctx;
			} *sl_MappedQoS_FlowsList_r16;
			struct SL_MappedQoS_FlowsListDedicated_r16	*sl_MappedQoS_FlowsListDedicated_r16;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *sl_MappedQoS_Flows_r16;
	long	*sl_CastType_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_SDAP_Config_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_SDAP_Header_r16_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_CastType_r16_10;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_SDAP_Config_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_SDAP_Config_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_SDAP_Config_r16_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SL-MappedQoS-FlowsListDedicated-r16.h"
#include "SL-QoS-Profile-r16.h"

#endif	/* _SL_SDAP_Config_r16_H_ */
#include <asn_internal.h>
