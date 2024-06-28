/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_DMRS_DownlinkConfig_H_
#define	_DMRS_DownlinkConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DMRS_DownlinkConfig__dmrs_Type {
	DMRS_DownlinkConfig__dmrs_Type_type2	= 0
} e_DMRS_DownlinkConfig__dmrs_Type;
typedef enum DMRS_DownlinkConfig__dmrs_AdditionalPosition {
	DMRS_DownlinkConfig__dmrs_AdditionalPosition_pos0	= 0,
	DMRS_DownlinkConfig__dmrs_AdditionalPosition_pos1	= 1,
	DMRS_DownlinkConfig__dmrs_AdditionalPosition_pos3	= 2
} e_DMRS_DownlinkConfig__dmrs_AdditionalPosition;
typedef enum DMRS_DownlinkConfig__maxLength {
	DMRS_DownlinkConfig__maxLength_len2	= 0
} e_DMRS_DownlinkConfig__maxLength;
typedef enum DMRS_DownlinkConfig__ext1__dmrs_Downlink_r16 {
	DMRS_DownlinkConfig__ext1__dmrs_Downlink_r16_enabled	= 0
} e_DMRS_DownlinkConfig__ext1__dmrs_Downlink_r16;

/* Forward declarations */
struct SetupRelease_PTRS_DownlinkConfig;

/* DMRS-DownlinkConfig */
typedef struct DMRS_DownlinkConfig {
	long	*dmrs_Type;	/* OPTIONAL */
	long	*dmrs_AdditionalPosition;	/* OPTIONAL */
	long	*maxLength;	/* OPTIONAL */
	long	*scramblingID0;	/* OPTIONAL */
	long	*scramblingID1;	/* OPTIONAL */
	struct SetupRelease_PTRS_DownlinkConfig	*phaseTrackingRS;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct DMRS_DownlinkConfig__ext1 {
		long	*dmrs_Downlink_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DMRS_DownlinkConfig_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_Type_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_AdditionalPosition_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_maxLength_8;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_Downlink_r16_15;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_DMRS_DownlinkConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_DMRS_DownlinkConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_DMRS_DownlinkConfig_1[7];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SetupRelease.h"

#endif	/* _DMRS_DownlinkConfig_H_ */
#include <asn_internal.h>
