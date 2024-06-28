/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-InterNodeDefinitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_AS_Context_H_
#define	_AS_Context_H_


#include <asn_application.h>

/* Including external dependencies */
#include <constr_SEQUENCE.h>
#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ReestablishmentInfo;
struct ConfigRestrictInfoSCG;
struct RAN_NotificationAreaInfo;
struct BandCombinationInfoSN;
struct ConfigRestrictInfoDAPS_r16;
struct NeedForGapsInfoNR_r16;
struct ConfigRestrictInfoDAPS_v1640;
struct NeedForGapNCSG_InfoNR_r17;
struct NeedForGapNCSG_InfoEUTRA_r17;

/* AS-Context */
typedef struct AS_Context {
	struct ReestablishmentInfo	*reestablishmentInfo;	/* OPTIONAL */
	struct ConfigRestrictInfoSCG	*configRestrictInfo;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct AS_Context__ext1 {
		struct RAN_NotificationAreaInfo	*ran_NotificationAreaInfo;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct AS_Context__ext2 {
		OCTET_STRING_t	*ueAssistanceInformation;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct AS_Context__ext3 {
		struct BandCombinationInfoSN	*selectedBandCombinationSN;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	struct AS_Context__ext4 {
		struct ConfigRestrictInfoDAPS_r16	*configRestrictInfoDAPS_r16;	/* OPTIONAL */
		OCTET_STRING_t	*sidelinkUEInformationNR_r16;	/* OPTIONAL */
		OCTET_STRING_t	*sidelinkUEInformationEUTRA_r16;	/* OPTIONAL */
		OCTET_STRING_t	*ueAssistanceInformationEUTRA_r16;	/* OPTIONAL */
		OCTET_STRING_t	*ueAssistanceInformationSCG_r16;	/* OPTIONAL */
		struct NeedForGapsInfoNR_r16	*needForGapsInfoNR_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext4;
	struct AS_Context__ext5 {
		struct ConfigRestrictInfoDAPS_v1640	*configRestrictInfoDAPS_v1640;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext5;
	struct AS_Context__ext6 {
		struct NeedForGapNCSG_InfoNR_r17	*needForGapNCSG_InfoNR_r17;	/* OPTIONAL */
		struct NeedForGapNCSG_InfoEUTRA_r17	*needForGapNCSG_InfoEUTRA_r17;	/* OPTIONAL */
		OCTET_STRING_t	*mbsInterestIndication_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext6;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} AS_Context_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AS_Context;
extern asn_SEQUENCE_specifics_t asn_SPC_AS_Context_specs_1;
extern asn_TYPE_member_t asn_MBR_AS_Context_1[8];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ReestablishmentInfo.h"
#include "ConfigRestrictInfoSCG.h"
#include "RAN-NotificationAreaInfo.h"
#include "BandCombinationInfoSN.h"
#include "ConfigRestrictInfoDAPS-r16.h"
#include "NeedForGapsInfoNR-r16.h"
#include "ConfigRestrictInfoDAPS-v1640.h"
#include "NeedForGapNCSG-InfoNR-r17.h"
#include "NeedForGapNCSG-InfoEUTRA-r17.h"

#endif	/* _AS_Context_H_ */
#include <asn_internal.h>
