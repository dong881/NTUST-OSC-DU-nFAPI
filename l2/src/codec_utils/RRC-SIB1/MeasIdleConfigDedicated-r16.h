/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MeasIdleConfigDedicated_r16_H_
#define	_MeasIdleConfigDedicated_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MeasIdleConfigDedicated_r16__measIdleDuration_r16 {
	MeasIdleConfigDedicated_r16__measIdleDuration_r16_sec10	= 0,
	MeasIdleConfigDedicated_r16__measIdleDuration_r16_sec30	= 1,
	MeasIdleConfigDedicated_r16__measIdleDuration_r16_sec60	= 2,
	MeasIdleConfigDedicated_r16__measIdleDuration_r16_sec120	= 3,
	MeasIdleConfigDedicated_r16__measIdleDuration_r16_sec180	= 4,
	MeasIdleConfigDedicated_r16__measIdleDuration_r16_sec240	= 5,
	MeasIdleConfigDedicated_r16__measIdleDuration_r16_sec300	= 6,
	MeasIdleConfigDedicated_r16__measIdleDuration_r16_spare	= 7
} e_MeasIdleConfigDedicated_r16__measIdleDuration_r16;

/* Forward declarations */
struct ValidityAreaList_r16;
struct MeasIdleCarrierNR_r16;
struct MeasIdleCarrierEUTRA_r16;

/* MeasIdleConfigDedicated-r16 */
typedef struct MeasIdleConfigDedicated_r16 {
	struct MeasIdleConfigDedicated_r16__measIdleCarrierListNR_r16 {
		A_SEQUENCE_OF(struct MeasIdleCarrierNR_r16) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *measIdleCarrierListNR_r16;
	struct MeasIdleConfigDedicated_r16__measIdleCarrierListEUTRA_r16 {
		A_SEQUENCE_OF(struct MeasIdleCarrierEUTRA_r16) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *measIdleCarrierListEUTRA_r16;
	long	 measIdleDuration_r16;
	struct ValidityAreaList_r16	*validityAreaList_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MeasIdleConfigDedicated_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_measIdleDuration_r16_6;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MeasIdleConfigDedicated_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_MeasIdleConfigDedicated_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_MeasIdleConfigDedicated_r16_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ValidityAreaList-r16.h"
#include "MeasIdleCarrierNR-r16.h"
#include "MeasIdleCarrierEUTRA-r16.h"

#endif	/* _MeasIdleConfigDedicated_r16_H_ */
#include <asn_internal.h>
