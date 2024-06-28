/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RAN_VisibleParameters_r17_H_
#define	_RAN_VisibleParameters_r17_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <BOOLEAN.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RAN_VisibleParameters_r17__ran_VisiblePeriodicity_r17 {
	RAN_VisibleParameters_r17__ran_VisiblePeriodicity_r17_ms120	= 0,
	RAN_VisibleParameters_r17__ran_VisiblePeriodicity_r17_ms240	= 1,
	RAN_VisibleParameters_r17__ran_VisiblePeriodicity_r17_ms480	= 2,
	RAN_VisibleParameters_r17__ran_VisiblePeriodicity_r17_ms640	= 3,
	RAN_VisibleParameters_r17__ran_VisiblePeriodicity_r17_ms1024	= 4
} e_RAN_VisibleParameters_r17__ran_VisiblePeriodicity_r17;

/* RAN-VisibleParameters-r17 */
typedef struct RAN_VisibleParameters_r17 {
	long	*ran_VisiblePeriodicity_r17;	/* OPTIONAL */
	long	*numberOfBufferLevelEntries_r17;	/* OPTIONAL */
	BOOLEAN_t	*reportPlayoutDelayForMediaStartup_r17;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RAN_VisibleParameters_r17_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_ran_VisiblePeriodicity_r17_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_RAN_VisibleParameters_r17;
extern asn_SEQUENCE_specifics_t asn_SPC_RAN_VisibleParameters_r17_specs_1;
extern asn_TYPE_member_t asn_MBR_RAN_VisibleParameters_r17_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _RAN_VisibleParameters_r17_H_ */
#include <asn_internal.h>
